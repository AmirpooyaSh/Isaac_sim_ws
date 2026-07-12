#!/usr/bin/env python3
"""
L-A-I: generate installation Sub-Steps for every element.

Place these files beside this script:
- Material.json
- Ext_A_II_Output.json
- Ext_A_III_Output.json
- config.json

Outputs:
- L_A_I_Output.json
- L_A_I_Run_Record.json
"""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import os
import platform
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from openai import OpenAI


ROOT = Path(__file__).resolve().parent
MATERIAL = ROOT / "Material.json"
EXT_II = ROOT / "Ext_A_II_Output.json"
EXT_III = ROOT / "Ext_A_III_Output.json"
CONFIG = ROOT / "config.json"
OUTPUT = ROOT / "L_A_I_Output.json"
RUN_RECORD = ROOT / "L_A_I_Run_Record.json"

TOOL_NAME = "submit_l_a_i_sub_steps"

TOOL = {
    "type": "function",
    "name": TOOL_NAME,
    "description": "Submit the ordered installation Sub-Steps for every element.",
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "SubSteps": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "Element": {"type": "string"},
                        "Steps": {
                            "type": "array",
                            "items": {"type": "string"}
                        }
                    },
                    "required": ["Element", "Steps"],
                    "additionalProperties": False
                }
            }
        },
        "required": ["SubSteps"],
        "additionalProperties": False
    }
}

SYSTEM_PROMPT = """
You are L-A-I in a construction knowledge-extraction pipeline.

Decompose the installation of every supplied assembly element into an ordered
list of human-understandable Sub-Steps (Si), using:
1. the Element Definition;
2. the element's assigned crews from Ext-A-II; and
3. the crew task descriptions from Ext-A-III.

Rules:
- Return exactly one mapping for every supplied element.
- Copy element names exactly.
- Give every element at least one Sub-Step.
- Write concise action phrases beginning with verbs.
- Keep steps in practical installation order.
- Adapt the crew-task knowledge to the specific element.
- Include only installation-relevant actions.
- Exclude estimating, scheduling, reporting, repair, and unrelated tasks.
- Do not invent dimensions, coordinates, tools, fastener counts, or tolerances.
- Do not include robotic commands or code.
- Do not repeat equivalent steps for the same element.
- Return only through the submit_l_a_i_sub_steps function.
""".strip()


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def save(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2, ensure_ascii=False)
        file.write("\n")


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def text_hash(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def unwrap(data: dict[str, Any], key: str) -> dict[str, Any]:
    return data[key] if key in data else data


def usage_dict(response: Any) -> dict[str, Any] | None:
    usage = getattr(response, "usage", None)
    return usage.model_dump(mode="json") if hasattr(usage, "model_dump") else None


def model_profile(config: dict[str, Any]) -> dict[str, Any]:
    return config["model_profiles"][config["active_model_profile"]]


def make_client(config: dict[str, Any]) -> OpenAI:
    api = config.get("api", {})
    options: dict[str, Any] = {
        "api_key": os.environ["OPENAI_API_KEY"],
        "timeout": api.get("timeout_seconds", 120),
        "max_retries": api.get("sdk_max_retries", 2)
    }

    base_url = os.getenv("OPENAI_BASE_URL") or api.get("base_url")
    if base_url:
        options["base_url"] = base_url

    return OpenAI(**options)


def make_request(config: dict[str, Any], prompt: str) -> dict[str, Any]:
    model = model_profile(config)

    request: dict[str, Any] = {
        "model": model["identifier"],
        "instructions": SYSTEM_PROMPT,
        "input": prompt,
        "tools": [TOOL],
        "tool_choice": {"type": "function", "name": TOOL_NAME},
        "parallel_tool_calls": False,
        "max_output_tokens": model.get("max_output_tokens", 4096),
        "store": config.get("request", {}).get("store", False)
    }

    if model.get("reasoning_effort") is not None:
        request["reasoning"] = {"effort": model["reasoning_effort"]}
    if model.get("temperature") is not None:
        request["temperature"] = model["temperature"]
    if model.get("top_p") is not None:
        request["top_p"] = model["top_p"]

    return request


def extract_arguments(response: Any) -> dict[str, Any]:
    for item in response.output:
        if item.type == "function_call" and item.name == TOOL_NAME:
            return json.loads(item.arguments)
    raise RuntimeError(f"The model did not call {TOOL_NAME}.")


def format_output(
    arguments: dict[str, Any],
    expected_elements: list[str]
) -> dict[str, Any]:
    returned: dict[str, list[str]] = {}

    for item in arguments.get("SubSteps", []):
        element = item.get("Element")
        steps = item.get("Steps")

        if not isinstance(element, str) or not isinstance(steps, list) or not steps:
            raise ValueError("Every element must have at least one Sub-Step.")

        unique_steps: list[str] = []
        seen: set[str] = set()

        for step in steps:
            if not isinstance(step, str) or not step.strip():
                raise ValueError(f"Invalid Sub-Step for {element!r}.")

            cleaned = " ".join(step.split())
            key = cleaned.casefold()

            if key not in seen:
                seen.add(key)
                unique_steps.append(cleaned)

        if element in returned:
            raise ValueError(f"Duplicate element: {element}")

        returned[element] = unique_steps

    if set(returned) != set(expected_elements):
        missing = sorted(set(expected_elements) - set(returned))
        extra = sorted(set(returned) - set(expected_elements))
        raise ValueError(f"Missing={missing}; additional={extra}")

    return {
        "Sub-Steps": [
            [element, returned[element]]
            for element in expected_elements
        ]
    }


def main() -> None:
    started = time.perf_counter()

    record: dict[str, Any] = {
        "agent": "L-A-I",
        "status": "running",
        "started_at_utc": now(),
        "completed_at_utc": None,
        "duration_seconds": None,
        "files": {
            "material_input": MATERIAL.name,
            "ext_a_ii_input": EXT_II.name,
            "ext_a_iii_input": EXT_III.name,
            "configuration": CONFIG.name,
            "output": OUTPUT.name,
            "run_record": RUN_RECORD.name
        },
        "attempts": []
    }

    try:
        if not os.getenv("OPENAI_API_KEY"):
            raise RuntimeError("OPENAI_API_KEY is not set.")

        material = load(MATERIAL)
        ext_ii = unwrap(load(EXT_II), "Ext-A-II")
        ext_iii = unwrap(load(EXT_III), "Ext-A-III")
        config = load(CONFIG)

        elements = material["Design"]
        crews = {element: labels for element, labels in ext_ii["Crews"]}
        tasks = {crew: descriptions for crew, descriptions in ext_iii["Tasks"]}

        expected = [item["Element"] for item in elements]

        grounded = []
        for definition in elements:
            name = definition["Element"]
            assigned_crews = crews.get(name, [])
            grounded.append({
                "ElementDefinition": definition,
                "AssignedCrews": assigned_crews,
                "CrewTasks": {
                    crew: tasks.get(crew, [None])
                    for crew in assigned_crews
                }
            })

        prompt = (
            "Generate installation Sub-Steps for every element below.\n\n"
            "Grounded inputs:\n"
            + json.dumps(grounded, indent=2, ensure_ascii=False)
            + "\n\nRequired element names:\n"
            + json.dumps(expected, indent=2, ensure_ascii=False)
        )

        model = model_profile(config)
        api = config.get("api", {})
        max_attempts = config.get("agent", {}).get(
            "validation_max_attempts", 3
        )
        request = make_request(config, prompt)

        record.update({
            "input_hashes_sha256": {
                MATERIAL.name: file_hash(MATERIAL),
                EXT_II.name: file_hash(EXT_II),
                EXT_III.name: file_hash(EXT_III),
                CONFIG.name: file_hash(CONFIG)
            },
            "element_count": len(expected),
            "expected_elements": expected,
            "grounded_inputs": grounded,
            "configuration": {
                "active_model_profile": config["active_model_profile"],
                "requested_model_identifier": model["identifier"],
                "api_family": api.get("api_family"),
                "api_version": api.get("api_version"),
                "reasoning_effort": model.get("reasoning_effort"),
                "temperature": model.get("temperature"),
                "top_p": model.get("top_p"),
                "max_output_tokens": model.get("max_output_tokens", 4096),
                "sdk_max_retries": api.get("sdk_max_retries", 2),
                "validation_max_attempts": max_attempts,
                "parallel_tool_calls": False,
                "strict_tool_schema": True,
                "tool_choice": TOOL_NAME
            },
            "software": {
                "python_version": platform.python_version(),
                "openai_python_version": importlib.metadata.version("openai")
            },
            "prompt_record": {
                "system_prompt": SYSTEM_PROMPT,
                "system_prompt_sha256": text_hash(SYSTEM_PROMPT),
                "user_prompt": prompt,
                "user_prompt_sha256": text_hash(prompt)
            },
            "tool_schema": TOOL
        })

        client = make_client(config)
        last_error = "Unknown validation failure."

        for attempt_number in range(1, max_attempts + 1):
            attempt_started = time.perf_counter()

            try:
                response = client.responses.create(**request)
                arguments = extract_arguments(response)
                final_output = format_output(arguments, expected)

                record["attempts"].append({
                    "attempt": attempt_number,
                    "status": "accepted",
                    "response_id": getattr(response, "id", None),
                    "returned_model_identifier": getattr(
                        response, "model", None
                    ),
                    "response_status": getattr(response, "status", None),
                    "duration_seconds": round(
                        time.perf_counter() - attempt_started, 6
                    ),
                    "usage": usage_dict(response),
                    "tool_arguments": arguments,
                    "formatted_output": final_output
                })

                save(OUTPUT, final_output)

                record.update({
                    "status": "success",
                    "completed_at_utc": now(),
                    "duration_seconds": round(
                        time.perf_counter() - started, 6
                    ),
                    "final_output": final_output
                })
                save(RUN_RECORD, record)

                print(json.dumps(final_output, indent=2, ensure_ascii=False))
                print(f"\nSaved: {OUTPUT}")
                print(f"Saved: {RUN_RECORD}")
                return

            except Exception as exc:
                last_error = f"{type(exc).__name__}: {exc}"
                record["attempts"].append({
                    "attempt": attempt_number,
                    "status": "rejected",
                    "duration_seconds": round(
                        time.perf_counter() - attempt_started, 6
                    ),
                    "error": last_error
                })

        raise RuntimeError(
            f"L-A-I failed after {max_attempts} attempts. {last_error}"
        )

    except Exception as exc:
        record.update({
            "status": "failed",
            "completed_at_utc": now(),
            "duration_seconds": round(time.perf_counter() - started, 6),
            "failure": {
                "type": type(exc).__name__,
                "message": str(exc)
            }
        })
        save(RUN_RECORD, record)
        raise


if __name__ == "__main__":
    main()
