#!/usr/bin/env python3
"""
Ext-A-I tool-calling agent.

Material.json is the Element Definitions input. The model is forced to call one
strict function whose arguments are the final Ext-A-I output:

    {
      "Structure": "Wall Panel",
      "Material": "Wood",
      "Confidence": 0.95
    }

The script separates:
1. SDK transport retries: network, timeout, rate-limit, and server failures.
2. Agent validation attempts: missing/invalid tool calls and low-confidence
   classifications.

It also writes a run record containing the exact model identifier, API family,
parameter values, SDK versions, tool schema, prompt hash, attempts, and token
usage for manuscript reproducibility.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import platform
import sys
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from openai import APIConnectionError, APIStatusError, APITimeoutError, OpenAI
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator


TOOL_NAME = "submit_ext_a_i_classification"
VAGUE_LABELS = {
    "",
    "unknown",
    "uncertain",
    "unspecified",
    "other",
    "n/a",
    "na",
    "none",
    "not sure",
}


# ---------------------------------------------------------------------------
# Input and output schemas
# ---------------------------------------------------------------------------

class ElementDefinition(BaseModel):
    model_config = ConfigDict(extra="forbid")

    Element: str = Field(min_length=1)
    Direction: list[str] = Field(min_length=1)
    Description: str = Field(min_length=1)


class MaterialFile(BaseModel):
    model_config = ConfigDict(extra="forbid")

    Design: list[ElementDefinition] = Field(min_length=1)


class ExtAIOutput(BaseModel):
    """Exact output format required by Ext-A-I."""

    model_config = ConfigDict(extra="forbid")

    Structure: str = Field(min_length=2)
    Material: str = Field(min_length=2)
    Confidence: float = Field(ge=0.0, le=1.0)

    @field_validator("Structure", "Material")
    @classmethod
    def reject_vague_labels(cls, value: str) -> str:
        cleaned = " ".join(value.split()).strip()
        if cleaned.casefold() in VAGUE_LABELS:
            raise ValueError("The classification must be specific and recognizable.")
        return cleaned


# ---------------------------------------------------------------------------
# Strict function/tool schema used as the output contract
# ---------------------------------------------------------------------------

EXT_AI_TOOL: dict[str, Any] = {
    "type": "function",
    "name": TOOL_NAME,
    "description": (
        "Submit Ext-A-I's final classification of the overall structural "
        "assembly and its dominant primary construction material."
    ),
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "Structure": {
                "type": "string",
                "description": (
                    "The overall structural assembly type, such as Wall Panel, "
                    "Roof Assembly, Floor System, or Ceiling Frame."
                ),
            },
            "Material": {
                "type": "string",
                "description": (
                    "The dominant primary construction material, such as Wood, "
                    "Steel, Concrete, or Composite."
                ),
            },
            "Confidence": {
                "type": "number",
                "minimum": 0.0,
                "maximum": 1.0,
                "description": (
                    "A numeric confidence score from 0 to 1 for the combined "
                    "Structure and Material classification."
                ),
            },
        },
        "required": ["Structure", "Material", "Confidence"],
        "additionalProperties": False,
    },
}


SYSTEM_PROMPT = """
You are Ext-A-I, the first classification agent in a construction knowledge-
extraction pipeline.

Examine the complete Element Definitions list and determine:
1. Structure: the overall structural assembly represented by the elements.
2. Material: the dominant primary construction material.
3. Confidence: a number from 0 to 1 representing certainty in both fields.

Rules:
- Use all element names, directions, and descriptions together.
- Base the result only on the supplied Element Definitions.
- Do not use the input filename as evidence.
- Return the result only by calling submit_ext_a_i_classification.
- Do not return explanatory prose outside the function call.
- The Structure and Material labels must be specific and recognizable.
- High confidence requires multiple mutually consistent definitions.
- Keep confidence below the acceptance threshold when evidence is vague,
  incomplete, or contradictory.
- Never increase confidence merely to pass validation.
""".strip()


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class AgentValidationError(RuntimeError):
    def __init__(self, code: str, message: str, candidate: dict[str, Any] | None = None):
        self.code = code
        self.candidate = candidate
        super().__init__(message)


class PipelineStoppedError(RuntimeError):
    def __init__(
        self,
        last_error: AgentValidationError,
        last_candidate: dict[str, Any] | None,
    ) -> None:
        self.last_error = last_error
        self.last_candidate = last_candidate
        super().__init__(
            f"Ext-A-I stopped after exhausting validation attempts: "
            f"{last_error.code}: {last_error}"
        )


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)
    except FileNotFoundError as exc:
        raise RuntimeError(f"File not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"Invalid JSON in {path}: line {exc.lineno}, column {exc.colno}."
        ) from exc

    if not isinstance(data, dict):
        raise RuntimeError(f"Expected a JSON object in {path}.")
    return data


def load_experiment_config(path: Path) -> dict[str, Any]:
    config = load_json(path)

    try:
        active_profile = config["active_model_profile"]
        profile = config["model_profiles"][active_profile]
        api = config["api"]
        agent = config["agent"]
    except KeyError as exc:
        raise RuntimeError(f"Missing configuration field: {exc}") from exc

    if not isinstance(active_profile, str) or not isinstance(profile, dict):
        raise RuntimeError("Invalid active_model_profile or model profile.")

    # Attach the selected profile without destroying the original config.
    resolved = deepcopy(config)
    resolved["resolved_model"] = deepcopy(profile)

    validate_experiment_config(resolved)
    return resolved


def validate_experiment_config(config: dict[str, Any]) -> None:
    api = config["api"]
    model = config["resolved_model"]
    agent = config["agent"]

    required_model_fields = {
        "identifier",
        "supports_reasoning_effort",
        "supports_sampling_controls",
        "reasoning_effort",
        "temperature",
        "top_p",
        "max_output_tokens",
    }
    missing = required_model_fields.difference(model)
    if missing:
        raise RuntimeError(f"Missing model-profile fields: {sorted(missing)}")

    if api.get("api_family") != "Responses API":
        raise RuntimeError("This script is implemented for the Responses API.")

    if api.get("api_version") != "v1":
        raise RuntimeError(
            "For the public OpenAI API, this script expects API version 'v1'."
        )

    threshold = float(agent["confidence_threshold"])
    if not 0.0 <= threshold <= 1.0:
        raise RuntimeError("confidence_threshold must be between 0 and 1.")

    if int(agent["validation_max_attempts"]) < 1:
        raise RuntimeError("validation_max_attempts must be at least 1.")

    if int(api["sdk_max_retries"]) < 0:
        raise RuntimeError("sdk_max_retries cannot be negative.")

    if float(api["timeout_seconds"]) <= 0:
        raise RuntimeError("timeout_seconds must be greater than 0.")

    max_tokens = int(model["max_output_tokens"])
    if max_tokens < 16:
        raise RuntimeError("max_output_tokens must be at least 16.")

    failure_handling = config.get("failure_handling", {})
    validation_actions = failure_handling.get("validation_actions", {})
    default_action = failure_handling.get("default_validation_action", "stop")
    allowed_actions = {"retry", "stop"}
    if default_action not in allowed_actions:
        raise RuntimeError("default_validation_action must be 'retry' or 'stop'.")
    invalid_actions = {
        code: action
        for code, action in validation_actions.items()
        if action not in allowed_actions
    }
    if invalid_actions:
        raise RuntimeError(
            f"Invalid validation failure actions: {invalid_actions}. "
            "Allowed values are 'retry' and 'stop'."
        )

    temperature = model.get("temperature")
    top_p = model.get("top_p")
    reasoning_effort = model.get("reasoning_effort")

    if temperature is not None and not 0.0 <= float(temperature) <= 2.0:
        raise RuntimeError("temperature must be null or between 0 and 2.")
    if top_p is not None and not 0.0 <= float(top_p) <= 1.0:
        raise RuntimeError("top_p must be null or between 0 and 1.")
    if temperature is not None and top_p is not None:
        raise RuntimeError(
            "Set either temperature or top_p, not both, for a controlled experiment."
        )

    if model["supports_reasoning_effort"]:
        if reasoning_effort is None:
            raise RuntimeError(
                "This model profile supports reasoning effort; specify it explicitly."
            )
    elif reasoning_effort is not None:
        raise RuntimeError(
            "reasoning_effort must be null for this non-reasoning model profile."
        )

    if not model["supports_sampling_controls"] and (
        temperature is not None or top_p is not None
    ):
        raise RuntimeError(
            "This model profile does not support temperature/top_p. Set both to null."
        )


def apply_cli_overrides(config: dict[str, Any], args: argparse.Namespace) -> None:
    model = config["resolved_model"]
    api = config["api"]
    agent = config["agent"]

    if args.model is not None:
        model["identifier"] = args.model
    if args.reasoning_effort is not None:
        model["reasoning_effort"] = (
            None if args.reasoning_effort == "omit" else args.reasoning_effort
        )
    if args.temperature is not None:
        model["temperature"] = args.temperature
    if args.top_p is not None:
        model["top_p"] = args.top_p
    if args.max_output_tokens is not None:
        model["max_output_tokens"] = args.max_output_tokens
    if args.sdk_max_retries is not None:
        api["sdk_max_retries"] = args.sdk_max_retries
    if args.validation_max_attempts is not None:
        agent["validation_max_attempts"] = args.validation_max_attempts
    if args.threshold is not None:
        agent["confidence_threshold"] = args.threshold
    if args.timeout is not None:
        api["timeout_seconds"] = args.timeout

    validate_experiment_config(config)


# ---------------------------------------------------------------------------
# Agent implementation
# ---------------------------------------------------------------------------

class ExtAIAgent:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        api = config["api"]
        model = config["resolved_model"]

        client_kwargs: dict[str, Any] = {
            "max_retries": int(api["sdk_max_retries"]),
            "timeout": float(api["timeout_seconds"]),
        }

        # Leave base_url null to use https://api.openai.com/v1.
        # OPENAI_BASE_URL overrides the JSON value for compatible gateways.
        base_url = os.getenv("OPENAI_BASE_URL") or api.get("base_url")
        if base_url:
            client_kwargs["base_url"] = base_url

        self.client = OpenAI(**client_kwargs)
        self.model = model
        self.threshold = float(config["agent"]["confidence_threshold"])
        self.validation_max_attempts = int(
            config["agent"]["validation_max_attempts"]
        )
        self.validation_retry_delay = float(
            config["agent"]["validation_retry_delay_seconds"]
        )
        failure_handling = config["failure_handling"]
        self.validation_actions = dict(
            failure_handling.get("validation_actions", {})
        )
        self.default_validation_action = failure_handling.get(
            "default_validation_action", "stop"
        )

    def _request_kwargs(self, user_prompt: str) -> dict[str, Any]:
        request: dict[str, Any] = {
            "model": self.model["identifier"],
            "instructions": SYSTEM_PROMPT,
            "input": user_prompt,
            "tools": [EXT_AI_TOOL],
            # Force exactly the output tool instead of allowing prose.
            "tool_choice": {"type": "function", "name": TOOL_NAME},
            "parallel_tool_calls": False,
            "max_output_tokens": int(self.model["max_output_tokens"]),
            "store": bool(self.config["request"]["store"]),
        }

        reasoning_effort = self.model.get("reasoning_effort")
        if reasoning_effort is not None:
            request["reasoning"] = {"effort": reasoning_effort}

        temperature = self.model.get("temperature")
        if temperature is not None:
            request["temperature"] = float(temperature)

        top_p = self.model.get("top_p")
        if top_p is not None:
            request["top_p"] = float(top_p)

        return request

    @staticmethod
    def _usage_to_dict(response: Any) -> dict[str, Any] | None:
        usage = getattr(response, "usage", None)
        if usage is None:
            return None
        if hasattr(usage, "model_dump"):
            return usage.model_dump(mode="json")
        return {
            key: getattr(usage, key)
            for key in dir(usage)
            if not key.startswith("_")
            and isinstance(getattr(usage, key, None), (int, float, str, bool, type(None)))
        }

    @staticmethod
    def _extract_tool_output(response: Any) -> tuple[ExtAIOutput, dict[str, Any]]:
        if getattr(response, "status", None) == "incomplete":
            details = getattr(response, "incomplete_details", None)
            reason = getattr(details, "reason", "unknown") if details else "unknown"
            raise AgentValidationError(
                "incomplete_response",
                f"The API response was incomplete: {reason}.",
            )

        function_calls = [
            item
            for item in getattr(response, "output", [])
            if getattr(item, "type", None) == "function_call"
        ]

        if not function_calls:
            raise AgentValidationError(
                "no_tool_call",
                f"The model did not call {TOOL_NAME}.",
            )

        if len(function_calls) != 1:
            raise AgentValidationError(
                "multiple_tool_calls",
                f"Expected one tool call but received {len(function_calls)}.",
            )

        tool_call = function_calls[0]
        if getattr(tool_call, "name", None) != TOOL_NAME:
            raise AgentValidationError(
                "wrong_tool_name",
                f"Expected {TOOL_NAME}, received {getattr(tool_call, 'name', None)!r}.",
            )

        raw_arguments = getattr(tool_call, "arguments", None)
        if not isinstance(raw_arguments, str):
            raise AgentValidationError(
                "missing_tool_arguments",
                "The function call did not contain JSON-encoded arguments.",
            )

        try:
            arguments = json.loads(raw_arguments)
        except json.JSONDecodeError as exc:
            raise AgentValidationError(
                "invalid_json_arguments",
                f"Tool arguments were not valid JSON: {exc}.",
            ) from exc

        try:
            result = ExtAIOutput.model_validate(arguments)
        except ValidationError as exc:
            raise AgentValidationError(
                "schema_validation_error",
                f"Tool arguments did not match the Ext-A-I schema: {exc}",
                candidate=arguments if isinstance(arguments, dict) else None,
            ) from exc

        call_metadata = {
            "call_id": getattr(tool_call, "call_id", None),
            "tool_name": getattr(tool_call, "name", None),
            "arguments": arguments,
        }
        return result, call_metadata

    def _validate_pipeline_gate(self, result: ExtAIOutput) -> None:
        if result.Confidence < self.threshold:
            raise AgentValidationError(
                "low_confidence",
                (
                    f"Confidence {result.Confidence:.3f} is below the required "
                    f"threshold {self.threshold:.3f}."
                ),
                candidate=result.model_dump(mode="json"),
            )

    def classify(
        self,
        elements: list[ElementDefinition],
        run_record: dict[str, Any],
    ) -> ExtAIOutput:
        element_definitions = [
            element.model_dump(mode="json") for element in elements
        ]

        validation_feedback = ""
        last_error: AgentValidationError | None = None
        last_candidate: dict[str, Any] | None = None

        for attempt_number in range(1, self.validation_max_attempts + 1):
            user_prompt = f"""
Element Definitions imported from Material.json:

{json.dumps(element_definitions, indent=2, ensure_ascii=False)}

Acceptance threshold: {self.threshold:.3f}.
{validation_feedback}
Call {TOOL_NAME} once with the final classification.
""".strip()

            attempt_record: dict[str, Any] = {
                "attempt_number": attempt_number,
                "started_at_utc": utc_now(),
                "status": "started",
            }
            run_record["attempts"].append(attempt_record)

            response = self.client.responses.create(**self._request_kwargs(user_prompt))

            attempt_record.update(
                {
                    "response_id": getattr(response, "id", None),
                    "response_model": getattr(response, "model", None),
                    "response_status": getattr(response, "status", None),
                    "usage": self._usage_to_dict(response),
                }
            )

            try:
                result, tool_metadata = self._extract_tool_output(response)
                attempt_record["tool_call"] = tool_metadata
                self._validate_pipeline_gate(result)
            except AgentValidationError as exc:
                last_error = exc
                last_candidate = exc.candidate
                attempt_record.update(
                    {
                        "status": "rejected",
                        "failure_code": exc.code,
                        "failure_message": str(exc),
                        "completed_at_utc": utc_now(),
                    }
                )

                action = self.validation_actions.get(
                    exc.code, self.default_validation_action
                )
                attempt_record["configured_action"] = action

                if action == "stop" or attempt_number >= self.validation_max_attempts:
                    break

                validation_feedback = f"""
Validation feedback from the previous attempt:
- Failure code: {exc.code}
- Failure reason: {exc}
- Previous candidate: {json.dumps(exc.candidate, ensure_ascii=False)}

Reassess the complete element list. Correct the result only when supported by
input evidence, and do not inflate Confidence to pass the threshold.
""".strip()

                if self.validation_retry_delay > 0:
                    time.sleep(self.validation_retry_delay)
                continue

            attempt_record.update(
                {
                    "status": "accepted",
                    "accepted_output": result.model_dump(mode="json"),
                    "completed_at_utc": utc_now(),
                }
            )
            return result

        assert last_error is not None
        raise PipelineStoppedError(last_error, last_candidate)


# ---------------------------------------------------------------------------
# I/O and reproducibility record
# ---------------------------------------------------------------------------

def load_element_definitions(path: Path) -> MaterialFile:
    raw_data = load_json(path)
    try:
        return MaterialFile.model_validate(raw_data)
    except ValidationError as exc:
        raise RuntimeError(
            f"{path} does not match the required Material.json schema:\n{exc}"
        ) from exc


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2, ensure_ascii=False)
        file.write("\n")


def package_version(package_name: str) -> str | None:
    try:
        return importlib.metadata.version(package_name)
    except importlib.metadata.PackageNotFoundError:
        return None


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def create_run_record(
    config: dict[str, Any],
    input_path: Path,
    config_path: Path,
) -> dict[str, Any]:
    published_config = deepcopy(config)

    # Avoid duplicating the selected profile under two keys in the record.
    published_config.pop("resolved_model", None)

    return {
        "run_started_at_utc": utc_now(),
        "status": "running",
        "input_file": str(input_path),
        "config_file": str(config_path),
        "active_model_profile": config["active_model_profile"],
        "resolved_model_configuration": deepcopy(config["resolved_model"]),
        "api_configuration": deepcopy(config["api"]),
        "agent_configuration": deepcopy(config["agent"]),
        "request_configuration": {
            **deepcopy(config["request"]),
            "endpoint": "/v1/responses",
            "tool_choice": {"type": "function", "name": TOOL_NAME},
            "parallel_tool_calls": False,
            "strict_tool_schema": True,
        },
        "failure_handling": deepcopy(config["failure_handling"]),
        "software": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "openai_sdk": package_version("openai"),
            "pydantic": package_version("pydantic"),
        },
        "prompt": {
            "system_prompt_sha256": sha256_text(SYSTEM_PROMPT),
            "system_prompt": SYSTEM_PROMPT,
        },
        "tool_schema": deepcopy(EXT_AI_TOOL),
        "attempts": [],
    }


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run Ext-A-I with forced strict tool calling and record the exact "
            "experimental configuration."
        )
    )
    parser.add_argument("--input", type=Path, default=Path("Material.json"))
    parser.add_argument(
        "--config", type=Path, default=Path("Ext_A_I_Config.json")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("Ext_A_I_Output.json")
    )
    parser.add_argument(
        "--failure-output", type=Path, default=Path("Ext_A_I_Failed.json")
    )
    parser.add_argument(
        "--run-record", type=Path, default=Path("Ext_A_I_Run_Record.json")
    )

    # Optional one-run overrides. JSON config remains the recommended source.
    parser.add_argument("--model")
    parser.add_argument(
        "--reasoning-effort",
        choices=["omit", "none", "minimal", "low", "medium", "high", "xhigh"],
    )
    parser.add_argument("--temperature", type=float)
    parser.add_argument("--top-p", type=float)
    parser.add_argument("--max-output-tokens", type=int)
    parser.add_argument("--sdk-max-retries", type=int)
    parser.add_argument("--validation-max-attempts", type=int)
    parser.add_argument("--threshold", type=float)
    parser.add_argument("--timeout", type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    if not os.getenv("OPENAI_API_KEY"):
        print(
            "ERROR: OPENAI_API_KEY is not set. Add it to ~/.bashrc and run "
            "'source ~/.bashrc'.",
            file=sys.stderr,
        )
        return 1

    run_record: dict[str, Any] | None = None

    try:
        config = load_experiment_config(args.config)
        apply_cli_overrides(config, args)
        material_data = load_element_definitions(args.input)
        run_record = create_run_record(config, args.input, args.config)

        agent = ExtAIAgent(config)
        result = agent.classify(material_data.Design, run_record)

        write_json(args.output, result.model_dump(mode="json"))
        run_record.update(
            {
                "status": "accepted",
                "final_output": result.model_dump(mode="json"),
                "run_completed_at_utc": utc_now(),
            }
        )
        write_json(args.run_record, run_record)

        print("Ext-A-I passed validation.")
        print(json.dumps(result.model_dump(mode="json"), indent=2))
        print(f"Output: {args.output}")
        print(f"Run record: {args.run_record}")
        return 0

    except PipelineStoppedError as exc:
        failure_data = {
            "status": "pipeline_stopped",
            "failure_code": exc.last_error.code,
            "failure_message": str(exc.last_error),
            "last_candidate": exc.last_candidate,
        }
        write_json(args.failure_output, failure_data)
        if run_record is not None:
            run_record.update(
                {
                    "status": "pipeline_stopped",
                    "final_failure": failure_data,
                    "run_completed_at_utc": utc_now(),
                }
            )
            write_json(args.run_record, run_record)

        print(f"ERROR: {exc}", file=sys.stderr)
        print(f"Failure record: {args.failure_output}", file=sys.stderr)
        return 2

    except (APIConnectionError, APITimeoutError, APIStatusError) as exc:
        # The SDK has already applied sdk_max_retries to retryable API failures.
        failure_data = {
            "status": "api_failure_after_sdk_retries",
            "error_type": type(exc).__name__,
            "message": str(exc),
        }
        write_json(args.failure_output, failure_data)
        if run_record is not None:
            run_record.update(
                {
                    "status": "api_failure_after_sdk_retries",
                    "final_failure": failure_data,
                    "run_completed_at_utc": utc_now(),
                }
            )
            write_json(args.run_record, run_record)

        print(f"ERROR: API request failed: {exc}", file=sys.stderr)
        return 3

    except (RuntimeError, ValueError) as exc:
        failure_data = {
            "status": "configuration_or_input_failure",
            "error_type": type(exc).__name__,
            "message": str(exc),
        }
        write_json(args.failure_output, failure_data)
        if run_record is not None:
            run_record.update(
                {
                    "status": "configuration_or_input_failure",
                    "final_failure": failure_data,
                    "run_completed_at_utc": utc_now(),
                }
            )
            write_json(args.run_record, run_record)

        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
