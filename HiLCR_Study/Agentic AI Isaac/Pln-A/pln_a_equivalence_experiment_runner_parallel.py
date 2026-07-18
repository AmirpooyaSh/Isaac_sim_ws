#!/usr/bin/env python3
"""
Run the repeated Pln-A equivalence experiment on three wall-panel complexity
levels under two information conditions.

Condition 1 - Framing Order + HLAFs + Human Knowledge + Design Info:
    Target assembly/design information, finalized HLAF headers/docstrings,
    Ext-A human trade knowledge, and explicit framing-order rules.

Condition 2 - HLAFs + Design Info + Worked Sample:
    Target assembly/design information, finalized HLAF headers/docstrings, and
    one worked sample consisting of Sample/1_Door_1_Window.json and
    Sample/1_Door_1_Window.txt. Human knowledge and framing-order rules are
    omitted from this condition.

By default, each condition is executed 30 times for each of the three cases:
No_Open, 1_Door, and 1_Door_1_Window. This produces 180 planned Pln-A runs.
For each repetition, the six case-condition combinations are submitted in one
parallel batch using up to six worker threads. Every run stores its prompt, raw
response, validated function sequence, and complete JSON run record in the same
directory structure used by the sequential runner. Completed successful runs
are skipped when the program is restarted unless --overwrite-existing is
supplied.

Expected directory structure beside this script:

    pln_a_equivalence_experiment_runner.py
    config.json
    Ext_A_II_Output.json
    Ext_A_III_Output.json
        or Ext_A_III_Output(1).json
    KBS_Simplified_Codified.py
    Framing_Order.txt
    Sample/
        1_Door_1_Window.json
        1_Door_1_Window.txt
    Examples/
        No_Open/No_Open.json
        1_Door/1_Door.json
        1_Door_1_Window/1_Door_1_Win.json

Required environment variable:
    OPENAI_API_KEY

Optional environment variables:
    OPENAI_BASE_URL   Overrides api.base_url from config.json.
    OPENAI_MODEL      Overrides the active profile's model identifier.
"""

from __future__ import annotations

import argparse
import ast
import csv
import threading
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
import hashlib
import json
import os
import random
import re
import shutil
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    from openai import OpenAI
except (ModuleNotFoundError, ImportError):
    OpenAI = None


BASE_DIR = Path(__file__).resolve().parent

DEFAULT_CONFIG_FILE = BASE_DIR / "config.json"
DEFAULT_CREWS_FILE = BASE_DIR / "Ext_A_II_Output.json"
DEFAULT_FUNCTIONS_FILE = BASE_DIR / "KBS_Simplified_Codified.py"
DEFAULT_FRAMING_ORDER_FILE = (
    BASE_DIR / "Framing_Order.txt"
)
DEFAULT_SAMPLE_ELEMENTS_FILE = (
    BASE_DIR / "Sample" / "1_Door_1_Window.json"
)
DEFAULT_SAMPLE_SEQUENCE_FILE = (
    BASE_DIR / "Sample" / "1_Door_1_Window.txt"
)
DEFAULT_OUTPUT_ROOT = BASE_DIR / "Pln_A_Equivalence_Results"

EXPERIMENT_CASES: tuple[tuple[str, Path], ...] = (
    ("No_Open", BASE_DIR / "Examples" / "No_Open" / "No_Open.json"),
    ("1_Door", BASE_DIR / "Examples" / "1_Door" / "1_Door.json"),
    (
        "1_Door_1_Window",
        BASE_DIR
        / "Examples"
        / "1_Door_1_Window"
        / "1_Door_1_Win.json",
    ),
)

@dataclass(frozen=True)
class ExperimentCondition:
    slug: str
    label: str
    description: str
    include_human_knowledge: bool
    include_design_rules: bool
    include_sample: bool


EXPERIMENT_CONDITIONS: tuple[ExperimentCondition, ...] = (
    ExperimentCondition(
        slug="1_Framing_Order_HLAFs_Human_Knowledge_Design",
        label="Framing Order + HLAFs + Human Knowledge + Design Info",
        description=(
            "Target assembly/design information and finalized HLAF interfaces "
            "are supplemented with Ext-A human trade knowledge and explicit "
            "framing-order/action-sequence rules. No worked sample is supplied."
        ),
        include_human_knowledge=True,
        include_design_rules=True,
        include_sample=False,
    ),
    ExperimentCondition(
        slug="2_HLAFs_Design_Info_With_Sample",
        label="HLAFs + Design Info + Worked Sample",
        description=(
            "Target assembly/design information and finalized HLAF interfaces "
            "are supplemented with one worked assembly example. Human trade "
            "knowledge and framing-order rules are omitted."
        ),
        include_human_knowledge=False,
        include_design_rules=False,
        include_sample=True,
    ),
)


@dataclass(frozen=True)
class FunctionSpec:
    name: str
    parameter_names: tuple[str, ...]
    parameters_with_none_default: tuple[str, ...]
    signature: str
    docstring: str


@dataclass(frozen=True)
class RuntimeConfig:
    model: str
    reasoning_effort: str
    max_output_tokens: int
    base_url: str
    timeout_seconds: float
    sdk_max_retries: int
    store: bool
    validation_max_attempts: int
    profile_name: str
    api_family: str
    api_version: str


@dataclass(frozen=True)
class ModelResult:
    content: str
    response_id: str | None
    model: str | None
    response_status: str | None
    incomplete_details: dict[str, Any] | None
    usage: dict[str, Any] | None


class AI_Agent:
    """Pln-A wrapper using the OpenAI Responses API."""

    def __init__(
        self,
        *,
        api_key: str,
        model: str,
        reasoning_effort: str,
        max_output_tokens: int,
        base_url: str,
        timeout_seconds: float,
        sdk_max_retries: int,
        store: bool,
    ) -> None:
        if OpenAI is None:
            raise RuntimeError(
                "The openai package is not installed. Run: pip install -U openai"
            )

        self.client = OpenAI(
            api_key=api_key,
            base_url=base_url,
            timeout=timeout_seconds,
            max_retries=sdk_max_retries,
        )
        self.model = model
        self.reasoning_effort = reasoning_effort
        self.max_output_tokens = max_output_tokens
        self.store = store

    def design_builder(self, information: str) -> ModelResult:
        submit_tool = {
            "type": "function",
            "name": "submit_pln_a_sequence",
            "description": (
                "Submit the complete Pln-A Python sequence as an ordered array "
                "of Python statements. Each array item must contain exactly one "
                "complete comment, direct HLAF call, or assignment receiving an "
                "HLAF return value."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "python_lines": {
                        "type": "array",
                        "description": (
                            "Ordered Python statements. Put one complete executable "
                            "statement or comment in each array item. Do not use Markdown "
                            "fences, headings, prose, imports, definitions, loops, "
                            "conditionals, literal-only assignments, or unsupported calls."
                        ),
                        "items": {"type": "string"},
                    }
                },
                "required": ["python_lines"],
                "additionalProperties": False,
            },
            "strict": True,
        }

        response = self.client.responses.create(
            model=self.model,
            reasoning={"effort": self.reasoning_effort},
            input=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "input_text",
                            "text": information,
                        }
                    ],
                }
            ],
            tools=[submit_tool],
            tool_choice={
                "type": "function",
                "name": "submit_pln_a_sequence",
            },
            parallel_tool_calls=False,
            max_output_tokens=self.max_output_tokens,
            store=self.store,
        )

        usage = getattr(response, "usage", None)
        usage_dict = model_dump_if_available(usage)
        incomplete_details = model_dump_if_available(
            getattr(response, "incomplete_details", None)
        )
        response_status = getattr(response, "status", None)

        # A tool call can be cut off mid-JSON when reasoning plus visible output
        # reaches max_output_tokens. Detect this before json.loads so the real
        # cause is reported instead of a generic invalid-JSON message.
        if response_status == "incomplete":
            reason = None
            if isinstance(incomplete_details, dict):
                reason = incomplete_details.get("reason")

            raw_arguments = get_raw_pln_a_arguments(response)
            partial_length = (
                len(raw_arguments) if isinstance(raw_arguments, str) else 0
            )
            raise RuntimeError(
                "The Responses API returned an incomplete tool call"
                + (f" because of {reason!r}" if reason else "")
                + f". Partial argument length: {partial_length} characters. "
                "Increase max_output_tokens in config.json (32768 is recommended "
                "for this high-reasoning, long-sequence request) or reduce the "
                "reasoning effort."
            )

        content = extract_pln_a_function_call(response)
        if not content.strip():
            raise RuntimeError(
                "The submit_pln_a_sequence tool returned an empty Python sequence."
            )

        return ModelResult(
            content=content.strip(),
            response_id=getattr(response, "id", None),
            model=getattr(response, "model", None),
            response_status=response_status,
            incomplete_details=incomplete_details,
            usage=usage_dict,
        )


def model_dump_if_available(value: Any) -> dict[str, Any] | None:
    if value is None:
        return None
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    if isinstance(value, dict):
        return value
    return None


def get_raw_pln_a_arguments(response: Any) -> str | dict[str, Any] | None:
    """Return raw arguments from the forced submit_pln_a_sequence call."""
    matching_calls: list[Any] = []

    for output_item in getattr(response, "output", []) or []:
        if (
            getattr(output_item, "type", None) == "function_call"
            and getattr(output_item, "name", None) == "submit_pln_a_sequence"
        ):
            matching_calls.append(output_item)

    if len(matching_calls) != 1:
        return None

    return getattr(matching_calls[0], "arguments", None)


def decode_tool_arguments(arguments: Any) -> dict[str, Any]:
    """Decode tool arguments and tolerate raw control characters.

    The official Responses API returns JSON-encoded function-call arguments.
    Some OpenAI-compatible gateways may place literal newlines in a JSON string,
    so a second parse uses strict=False. Truncated JSON is still rejected.
    """
    if isinstance(arguments, dict):
        return arguments

    if not isinstance(arguments, str):
        raise RuntimeError(
            "The submit_pln_a_sequence function call did not contain JSON arguments."
        )

    try:
        payload = json.loads(arguments)
    except json.JSONDecodeError as first_error:
        try:
            payload = json.loads(arguments, strict=False)
        except json.JSONDecodeError as second_error:
            tail = arguments[-500:]
            raise RuntimeError(
                "The submit_pln_a_sequence arguments were not valid JSON. "
                f"Decoder error: {second_error.msg} at line "
                f"{second_error.lineno}, column {second_error.colno}, "
                f"character {second_error.pos}. Argument length: "
                f"{len(arguments)} characters. Argument tail: {tail!r}. "
                "This usually means the tool call was truncated or the API "
                "gateway returned malformed function-call arguments."
            ) from first_error

    if not isinstance(payload, dict):
        raise RuntimeError(
            "The submit_pln_a_sequence arguments decoded successfully, but "
            "the decoded value was not a JSON object."
        )

    return payload


def extract_pln_a_function_call(response: Any) -> str:
    """Extract Python statements from the forced Pln-A function call."""
    matching_calls: list[Any] = []

    for output_item in getattr(response, "output", []) or []:
        if (
            getattr(output_item, "type", None) == "function_call"
            and getattr(output_item, "name", None) == "submit_pln_a_sequence"
        ):
            matching_calls.append(output_item)

    if len(matching_calls) != 1:
        raise RuntimeError(
            "Expected exactly one submit_pln_a_sequence function call, "
            f"but received {len(matching_calls)}."
        )

    arguments = getattr(matching_calls[0], "arguments", None)
    payload = decode_tool_arguments(arguments)

    # New representation: one complete statement per JSON array item.
    python_lines = payload.get("python_lines")
    if isinstance(python_lines, list):
        if not python_lines:
            raise RuntimeError(
                "The submit_pln_a_sequence arguments contained an empty "
                "python_lines array."
            )
        if not all(isinstance(line, str) for line in python_lines):
            raise RuntimeError("Every python_lines item must be a string.")
        return "\n".join(line.rstrip() for line in python_lines).strip()

    # Backward-compatible fallback for responses produced by the older schema.
    python_code = payload.get("python_code")
    if isinstance(python_code, str):
        return python_code

    raise RuntimeError(
        "The submit_pln_a_sequence arguments did not contain a valid "
        "python_lines array."
    )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(value, file, indent=2, ensure_ascii=False)
        file.write("\n")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = BASE_DIR / path
    return path.resolve()


def load_runtime_config(
    config_path: Path,
    *,
    model_override: str | None = None,
) -> RuntimeConfig:
    config = read_json(config_path)

    if not isinstance(config, dict):
        raise ValueError("The configuration root must be a JSON object.")

    profile_name = config.get("active_model_profile")
    profiles = config.get("model_profiles")
    api_config = config.get("api", {})
    request_config = config.get("request", {})
    agent_config = config.get("agent", {})

    if not isinstance(profile_name, str) or not profile_name:
        raise ValueError("config.json requires active_model_profile.")
    if not isinstance(profiles, dict) or profile_name not in profiles:
        raise ValueError(
            f"Model profile {profile_name!r} is missing from model_profiles."
        )

    profile = profiles[profile_name]
    if not isinstance(profile, dict):
        raise ValueError("The active model profile must be a JSON object.")

    configured_identifier = profile.get("identifier")
    model = (
        model_override
        or os.getenv("OPENAI_MODEL")
        or configured_identifier
    )
    if not isinstance(model, str) or not model:
        raise ValueError("A model identifier must be supplied.")

    reasoning_effort = profile.get("reasoning_effort", "high")
    if reasoning_effort not in {"low", "medium", "high"}:
        raise ValueError(
            "reasoning_effort must be one of: low, medium, high."
        )

    max_output_tokens = profile.get("max_output_tokens", 32768)
    if not isinstance(max_output_tokens, int) or max_output_tokens < 1:
        raise ValueError("max_output_tokens must be a positive integer.")

    if not isinstance(api_config, dict):
        raise ValueError("api must be a JSON object.")

    configured_base_url = api_config.get("base_url")
    base_url = (
        os.getenv("OPENAI_BASE_URL")
        or configured_base_url
        or "https://api.openai.com/v1"
    )

    timeout_seconds = api_config.get("timeout_seconds", 300)
    sdk_max_retries = api_config.get("sdk_max_retries", 2)
    validation_max_attempts = agent_config.get(
        "validation_max_attempts", 3
    )
    store = request_config.get("store", False)

    if not isinstance(timeout_seconds, (int, float)) or timeout_seconds <= 0:
        raise ValueError("timeout_seconds must be positive.")
    if not isinstance(sdk_max_retries, int) or sdk_max_retries < 0:
        raise ValueError("sdk_max_retries must be a nonnegative integer.")
    if (
        not isinstance(validation_max_attempts, int)
        or validation_max_attempts < 1
    ):
        raise ValueError(
            "validation_max_attempts must be a positive integer."
        )
    if not isinstance(store, bool):
        raise ValueError("request.store must be true or false.")

    return RuntimeConfig(
        model=model,
        reasoning_effort=reasoning_effort,
        max_output_tokens=max_output_tokens,
        base_url=str(base_url),
        timeout_seconds=float(timeout_seconds),
        sdk_max_retries=sdk_max_retries,
        store=store,
        validation_max_attempts=validation_max_attempts,
        profile_name=profile_name,
        api_family=str(api_config.get("api_family", "Responses API")),
        api_version=str(api_config.get("api_version", "v1")),
    )


def find_tasks_file() -> Path:
    preferred = (
        BASE_DIR / "Ext_A_III_Output.json",
        BASE_DIR / "Ext_A_III_Output(1).json",
    )
    for path in preferred:
        if path.exists():
            return path

    candidates = sorted(BASE_DIR.glob("Ext_A_III_Output*.json"))
    if not candidates:
        raise FileNotFoundError("No Ext_A_III_Output*.json file was found.")
    if len(candidates) > 1:
        names = ", ".join(path.name for path in candidates)
        raise RuntimeError(
            "Multiple Ext-A-III files were found: "
            f"{names}. Rename the intended file to Ext_A_III_Output.json."
        )
    return candidates[0]


def validate_required_files(
    *,
    config_file: Path,
    crews_file: Path,
    tasks_file: Path,
    functions_file: Path,
    framing_order_file: Path,
    sample_elements_file: Path,
    sample_sequence_file: Path,
) -> None:
    required = [
        config_file,
        crews_file,
        tasks_file,
        functions_file,
        framing_order_file,
        sample_elements_file,
        sample_sequence_file,
        *[case_path for _, case_path in EXPERIMENT_CASES],
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing required files:\n- " + "\n- ".join(missing))


def placeholder_signature(
    node: ast.FunctionDef | ast.AsyncFunctionDef,
) -> str:
    if isinstance(node, ast.AsyncFunctionDef):
        placeholder: ast.AST = ast.AsyncFunctionDef(
            name=node.name,
            args=node.args,
            body=[ast.Pass()],
            decorator_list=[],
            returns=node.returns,
            type_comment=node.type_comment,
        )
    else:
        placeholder = ast.FunctionDef(
            name=node.name,
            args=node.args,
            body=[ast.Pass()],
            decorator_list=[],
            returns=node.returns,
            type_comment=node.type_comment,
        )

    ast.fix_missing_locations(placeholder)
    rendered = ast.unparse(placeholder)
    signature_lines: list[str] = []
    for line in rendered.splitlines():
        if line.strip() == "pass":
            break
        signature_lines.append(line)
    return "\n".join(signature_lines).strip()


def function_parameter_names(
    node: ast.FunctionDef | ast.AsyncFunctionDef,
) -> tuple[str, ...]:
    names = [
        argument.arg
        for argument in (
            *node.args.posonlyargs,
            *node.args.args,
            *node.args.kwonlyargs,
        )
        if argument.arg != "self"
    ]
    if node.args.vararg is not None:
        names.append(node.args.vararg.arg)
    if node.args.kwarg is not None:
        names.append(node.args.kwarg.arg)
    return tuple(names)


def parameters_with_none_default(
    node: ast.FunctionDef | ast.AsyncFunctionDef,
) -> tuple[str, ...]:
    positional = [*node.args.posonlyargs, *node.args.args]
    default_by_name: dict[str, ast.expr | None] = {
        argument.arg: None for argument in positional
    }

    if node.args.defaults:
        defaulted_arguments = positional[-len(node.args.defaults) :]
        for argument, default in zip(defaulted_arguments, node.args.defaults):
            default_by_name[argument.arg] = default

    required: list[str] = []
    for argument in positional:
        if argument.arg == "self":
            continue
        default = default_by_name[argument.arg]
        if default is None or (
            isinstance(default, ast.Constant) and default.value is None
        ):
            required.append(argument.arg)

    for argument, default in zip(
        node.args.kwonlyargs,
        node.args.kw_defaults,
    ):
        if default is None or (
            isinstance(default, ast.Constant) and default.value is None
        ):
            required.append(argument.arg)

    return tuple(required)


def extract_hlaf_catalog(
    path: Path,
) -> tuple[str, dict[str, FunctionSpec]]:
    source = read_text(path)
    tree = ast.parse(source, filename=str(path))

    entries: list[str] = []
    specs: dict[str, FunctionSpec] = {}

    for node in tree.body:
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue

        signature = placeholder_signature(node)
        docstring = ast.get_docstring(node, clean=False)
        if not docstring:
            docstring = "No descriptive docstring is available."

        spec = FunctionSpec(
            name=node.name,
            parameter_names=function_parameter_names(node),
            parameters_with_none_default=parameters_with_none_default(node),
            signature=signature,
            docstring=docstring.strip(),
        )
        specs[node.name] = spec

        entries.append(
            "\n".join(
                [
                    f"Function Name: {node.name}",
                    "Signature:",
                    signature,
                    "Description:",
                    docstring.strip(),
                ]
            )
        )

    if not entries:
        raise ValueError(f"No top-level functions were found in {path.name}.")

    return "\n\n".join(entries), specs


def normalize_crews(data: Any) -> list[Any]:
    if not isinstance(data, dict) or not isinstance(data.get("Crews"), list):
        raise ValueError(
            "Ext_A_II_Output.json must contain a top-level Crews list."
        )
    return data["Crews"]


def normalize_tasks(data: Any) -> list[Any]:
    if not isinstance(data, dict) or not isinstance(data.get("Tasks"), list):
        raise ValueError(
            "Ext-A-III output must contain a top-level Tasks list."
        )
    return data["Tasks"]


def strip_code_fences(value: str) -> str:
    stripped = value.strip()
    match = re.fullmatch(
        r"```(?:python)?\s*(.*?)\s*```",
        stripped,
        flags=re.DOTALL | re.IGNORECASE,
    )
    return match.group(1).strip() if match else stripped


def condition_context(
    *,
    condition: ExperimentCondition,
    design_rules: str,
    crews: list[Any],
    tasks: list[Any],
    sample_assembly: Any,
    sample_sequence: str,
) -> str:
    sections: list[str] = []

    if condition.include_design_rules:
        sections.append(
            "Design Rules / Framing Order:\n"
            f"{design_rules.strip()}"
        )
    else:
        sections.append(
            "Design Rules / Framing Order:\n"
            "Not supplied in this condition. Do not assume access to omitted "
            "framing-order or action-sequence rules."
        )

    if condition.include_human_knowledge:
        sections.append(
            "Human Trade Knowledge:\n"
            "Use these records as construction-process context when selecting "
            "and ordering HLAFs.\n\n"
            "Component-to-Crew Mapping:\n"
            f"{json.dumps(crews, indent=2, ensure_ascii=False)}\n\n"
            "Crew-to-Task Knowledge:\n"
            f"{json.dumps(tasks, indent=2, ensure_ascii=False)}"
        )
    else:
        sections.append(
            "Human Trade Knowledge:\n"
            "Not supplied in this condition. Do not assume access to omitted "
            "crew mappings or occupational task descriptions."
        )

    if condition.include_sample:
        sections.append(
            "Worked Sample Assembly Instance:\n"
            f"{json.dumps(sample_assembly, indent=2, ensure_ascii=False)}\n\n"
            "Worked Sample Function Sequence:\n"
            f"{sample_sequence.strip()}\n\n"
            "Use the sample to infer reusable element-to-function mappings, "
            "operation order, return-value dependencies, and repetition patterns. "
            "Adapt those patterns to the target assembly rather than copying sample "
            "element names or coordinates. The sample may contain positional calls "
            "and the legacy helper Drag_Stud, which is not part of the current HLAF "
            "catalog. Do not emit Drag_Stud or any other unsupported function. The "
            "generated target sequence must use only current HLAF names and exact "
            "keyword arguments from the supplied catalog."
        )
    else:
        sections.append(
            "Worked Sample:\n"
            "Not supplied in this condition."
        )

    return "\n\n".join(sections)


def build_prompt(
    *,
    case_name: str,
    assembly_instance: Any,
    hlaf_catalog: str,
    condition: ExperimentCondition,
    design_rules: str,
    crews: list[Any],
    tasks: list[Any],
    sample_assembly: Any,
    sample_sequence: str,
    validation_feedback: str = "",
) -> str:
    prompt = (
        "You are Pln-A, a construction robotic planning agent. Generate an "
        "ordered sequence of calls to the supplied finalized high-level "
        "assembly functions for the target wall-panel case "
        f"{case_name!r}.\n\n"
        f"Experimental condition: {condition.label}.\n"
        f"Condition definition: {condition.description}\n\n"
        "The Target Assembly Instance Data contains design-specific element "
        "names, types, coordinates, dimensions, and geometric metadata. Use "
        "exact values from this JSON when populating function arguments. Do "
        "not invent missing element geometry.\n\n"
        "The HLAF catalog contains only function headers and docstrings. These "
        "are the only functions that may be called. Infer each function's "
        "capability, required inputs, sequencing dependencies, and return-value "
        "usage from its signature and docstring. Use exact function names and "
        "exact keyword argument names. Do not define new functions, import "
        "modules, call low-level robot methods, or include unsupported helpers.\n\n"
        "Submit the complete sequence through the required submit_pln_a_sequence "
        "tool. Its python_lines field must be an ordered JSON array containing "
        "one complete Python statement per item. Each item may contain only: "
        "(1) one Python comment, (2) one direct HLAF call, or (3) one simple "
        "assignment whose right-hand side is one direct HLAF call and whose "
        "returned value is used later. Keep each HLAF call in a single array "
        "item even when it is long. Do not include headings as string literals, "
        "standalone prose, imports, literal variable initializations, standalone "
        "dictionaries or lists, print calls, new function or class definitions, "
        "loops, conditionals, try blocks, or Markdown fences. Literal lists are "
        "allowed only inside HLAF keyword arguments when required by a documented "
        "parameter such as el_pose or el_dims. Use keyword arguments rather than "
        "positional arguments. Optional parameters may be omitted when their "
        "documented defaults are appropriate. Every executable item must therefore "
        "have one of these forms: HLAF_Name(keyword=value, ...) or result_name = "
        "HLAF_Name(keyword=value, ...).\n\n"
        "Target Assembly Instance Data:\n"
        f"{json.dumps(assembly_instance, indent=2, ensure_ascii=False)}\n\n"
        "Finalized HLAF Headers and Docstrings:\n"
        f"{hlaf_catalog}\n\n"
        f"{condition_context(condition=condition, design_rules=design_rules, crews=crews, tasks=tasks, sample_assembly=sample_assembly, sample_sequence=sample_sequence)}"
    )

    if validation_feedback:
        prompt += (
            "\n\nThe previous generated sequence was rejected by the local "
            "validator for this reason:\n"
            f"{validation_feedback}\n"
            "Return a corrected complete sequence for the same target case and "
            "experimental condition."
        )

    return prompt


def function_call_name(call: ast.Call) -> str:
    if not isinstance(call.func, ast.Name):
        raise ValueError(
            "Every call must directly invoke a supplied HLAF function by name."
        )
    return call.func.id


def validate_call(
    call: ast.Call,
    specs: dict[str, FunctionSpec],
    assigned_names: set[str],
) -> str:
    function_name = function_call_name(call)
    if function_name not in specs:
        raise ValueError(f"Unsupported function call: {function_name!r}.")
    if call.args:
        raise ValueError(
            f"{function_name} uses positional arguments. Only keyword arguments are allowed."
        )
    if any(keyword.arg is None for keyword in call.keywords):
        raise ValueError(
            f"{function_name} uses **kwargs expansion, which is not allowed."
        )

    keyword_names = [
        keyword.arg for keyword in call.keywords if keyword.arg is not None
    ]
    if len(keyword_names) != len(set(keyword_names)):
        raise ValueError(f"{function_name} repeats a keyword argument.")

    spec = specs[function_name]
    unknown = sorted(set(keyword_names) - set(spec.parameter_names))
    if unknown:
        raise ValueError(
            f"{function_name} uses unknown keyword arguments: {unknown}."
        )

    missing_none_parameters = sorted(
        set(spec.parameters_with_none_default) - set(keyword_names)
    )
    if missing_none_parameters:
        raise ValueError(
            f"{function_name} omits parameters whose declared default is None: "
            f"{missing_none_parameters}."
        )

    for keyword in call.keywords:
        for child in ast.walk(keyword.value):
            if isinstance(child, ast.Call):
                raise ValueError(
                    f"{function_name} contains a nested function call inside an argument."
                )
            if isinstance(child, ast.Name) and (
                child.id not in assigned_names
                and child.id not in {"True", "False", "None"}
            ):
                raise ValueError(
                    f"{function_name} references variable {child.id!r} before it "
                    "is assigned by a previous HLAF call."
                )

    return function_name


def validate_generated_sequence(
    generated_code: str,
    specs: dict[str, FunctionSpec],
) -> dict[str, Any]:
    tree = ast.parse(generated_code, filename="<Pln-A generated sequence>")
    assigned_names: set[str] = set()
    call_names: list[str] = []

    for statement in tree.body:
        call: ast.Call

        if isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Call):
            call = statement.value
        elif isinstance(statement, ast.Assign) and isinstance(statement.value, ast.Call):
            if len(statement.targets) != 1:
                raise ValueError(
                    "A returned HLAF value may be assigned to only one variable."
                )
            target = statement.targets[0]
            if not isinstance(target, ast.Name):
                raise ValueError(
                    "HLAF return values must be assigned to a simple variable name."
                )
            call = statement.value
            function_name = validate_call(call, specs, assigned_names)
            call_names.append(function_name)
            assigned_names.add(target.id)
            continue
        elif isinstance(statement, ast.AnnAssign) and isinstance(
            statement.value, ast.Call
        ):
            if not isinstance(statement.target, ast.Name):
                raise ValueError(
                    "Annotated HLAF return values must be assigned to a simple variable name."
                )
            call = statement.value
            function_name = validate_call(call, specs, assigned_names)
            call_names.append(function_name)
            assigned_names.add(statement.target.id)
            continue
        else:
            try:
                rendered_statement = ast.unparse(statement).strip()
            except Exception:
                rendered_statement = "<unable to render statement>"

            if len(rendered_statement) > 300:
                rendered_statement = rendered_statement[:297] + "..."

            raise ValueError(
                "Unsupported top-level statement at "
                f"line {getattr(statement, 'lineno', '?')}: "
                f"{type(statement).__name__}: {rendered_statement!r}. "
                "Only direct HLAF calls, assignments receiving an HLAF return "
                "value, and comments are allowed."
            )

        function_name = validate_call(call, specs, assigned_names)
        call_names.append(function_name)

    if not call_names:
        raise ValueError("The generated sequence contains no HLAF calls.")

    return {
        "call_count": len(call_names),
        "function_calls": call_names,
        "unique_function_calls": sorted(set(call_names)),
        "assigned_result_variables": sorted(assigned_names),
    }


def execute_single_run(
    *,
    agent: AI_Agent,
    runtime_config: RuntimeConfig,
    case_name: str,
    assembly_path: Path,
    condition: ExperimentCondition,
    repetition_index: int,
    execution_order_index: int,
    output_root: Path,
    design_rules: str,
    hlaf_catalog: str,
    function_specs: dict[str, FunctionSpec],
    crews: list[Any],
    tasks: list[Any],
    sample_assembly: Any,
    sample_sequence: str,
    tasks_file: Path,
    crews_file: Path,
    functions_file: Path,
    framing_order_file: Path,
    sample_elements_file: Path,
    sample_sequence_file: Path,
    max_attempts: int,
    save_prompt: bool,
) -> dict[str, Any]:
    run_started = time.perf_counter()
    started_at = utc_now()

    run_dir = (
        output_root
        / case_name
        / condition.slug
        / f"Run_{repetition_index:03d}"
    )
    run_dir.mkdir(parents=True, exist_ok=True)

    sequence_path = run_dir / "Final_Function_Sequence.py"
    record_path = run_dir / "Run_Record.json"
    prompt_path = run_dir / "Prompt.txt"
    raw_response_path = run_dir / "Raw_Response.txt"

    assembly_instance = read_json(assembly_path)

    input_files: dict[str, str] = {
        assembly_path.name: sha256_file(assembly_path),
        functions_file.name: sha256_file(functions_file),
    }
    if condition.include_design_rules:
        input_files[framing_order_file.name] = sha256_file(
            framing_order_file
        )
    if condition.include_human_knowledge:
        input_files.update(
            {
                crews_file.name: sha256_file(crews_file),
                tasks_file.name: sha256_file(tasks_file),
            }
        )
    if condition.include_sample:
        input_files.update(
            {
                f"Sample/{sample_elements_file.name}": sha256_file(
                    sample_elements_file
                ),
                f"Sample/{sample_sequence_file.name}": sha256_file(
                    sample_sequence_file
                ),
            }
        )

    run_record: dict[str, Any] = {
        "agent": "Pln-A",
        "case": case_name,
        "condition": condition.slug,
        "condition_label": condition.label,
        "condition_description": condition.description,
        "repetition_index": repetition_index,
        "execution_order_index": execution_order_index,
        "human_knowledge_included": condition.include_human_knowledge,
        "design_rules_included": condition.include_design_rules,
        "sample_included": condition.include_sample,
        "status": "running",
        "started_at_utc": started_at,
        "completed_at_utc": None,
        "duration_seconds": None,
        "model_configuration": {
            "profile": runtime_config.profile_name,
            "model": runtime_config.model,
            "api_family": runtime_config.api_family,
            "api_version": runtime_config.api_version,
            "base_url": runtime_config.base_url,
            "reasoning_effort": runtime_config.reasoning_effort,
            "max_output_tokens": runtime_config.max_output_tokens,
            "temperature": None,
            "top_p": None,
            "timeout_seconds": runtime_config.timeout_seconds,
            "sdk_max_retries": runtime_config.sdk_max_retries,
            "store": runtime_config.store,
            "validation_max_attempts": max_attempts,
        },
        "input_files_sha256": input_files,
        "output_files": {
            "function_sequence": sequence_path.name,
            "run_record": record_path.name,
            "prompt": prompt_path.name if save_prompt else None,
            "raw_response": raw_response_path.name if save_prompt else None,
        },
        "attempts": [],
    }
    write_json(record_path, run_record)

    feedback = ""
    last_error = "Unknown generation error."

    for attempt_number in range(1, max_attempts + 1):
        attempt_started = time.perf_counter()
        prompt = build_prompt(
            case_name=case_name,
            assembly_instance=assembly_instance,
            hlaf_catalog=hlaf_catalog,
            condition=condition,
            design_rules=design_rules,
            crews=crews,
            tasks=tasks,
            sample_assembly=sample_assembly,
            sample_sequence=sample_sequence,
            validation_feedback=feedback,
        )

        result: ModelResult | None = None
        generated_code: str | None = None

        try:
            result = agent.design_builder(prompt)
            generated_code = strip_code_fences(result.content)

            if save_prompt:
                attempt_prompt_path = run_dir / f"Attempt_{attempt_number:02d}_Prompt.txt"
                attempt_raw_path = run_dir / f"Attempt_{attempt_number:02d}_Raw_Response.txt"
                attempt_prompt_path.write_text(prompt, encoding="utf-8")
                attempt_raw_path.write_text(result.content + "\n", encoding="utf-8")

            validation = validate_generated_sequence(
                generated_code,
                function_specs,
            )

            generated_code = (
                f"# Pln-A case: {case_name}\n"
                f"# Experimental condition: {condition.slug}\n"
                f"# Repetition: {repetition_index:03d}\n"
                f"# Execution order: {execution_order_index:03d}\n\n"
                + generated_code.rstrip()
                + "\n"
            )
            compile(generated_code, str(sequence_path), "exec")
            sequence_path.write_text(generated_code, encoding="utf-8")

            if save_prompt:
                prompt_path.write_text(prompt, encoding="utf-8")
                raw_response_path.write_text(
                    result.content + "\n",
                    encoding="utf-8",
                )

            attempt_record = {
                "attempt": attempt_number,
                "status": "accepted",
                "response_id": result.response_id,
                "returned_model": result.model,
                "response_status": result.response_status,
                "incomplete_details": result.incomplete_details,
                "usage": result.usage,
                "duration_seconds": round(
                    time.perf_counter() - attempt_started,
                    6,
                ),
                "prompt_sha256": sha256_text(prompt),
                "raw_response_sha256": sha256_text(result.content),
                "validation": validation,
            }
            run_record["attempts"].append(attempt_record)

            run_record.update(
                {
                    "status": "success",
                    "completed_at_utc": utc_now(),
                    "duration_seconds": round(
                        time.perf_counter() - run_started,
                        6,
                    ),
                    "final_sequence_sha256": sha256_file(sequence_path),
                    "final_validation": validation,
                }
            )
            write_json(record_path, run_record)

            print(
                f"[Pln-A] {case_name} / {condition.slug} / "
                f"Run_{repetition_index:03d}: "
                f"success ({validation['call_count']} HLAF calls)"
            )
            return run_record

        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"

            if save_prompt:
                attempt_prompt_path = (
                    run_dir / f"Attempt_{attempt_number:02d}_Prompt.txt"
                )
                attempt_prompt_path.write_text(prompt, encoding="utf-8")

            rejected_attempt: dict[str, Any] = {
                "attempt": attempt_number,
                "status": "rejected",
                "duration_seconds": round(
                    time.perf_counter() - attempt_started,
                    6,
                ),
                "error": last_error,
                "prompt_sha256": sha256_text(prompt),
            }

            if result is not None:
                rejected_attempt.update(
                    {
                        "response_id": result.response_id,
                        "returned_model": result.model,
                        "response_status": result.response_status,
                        "incomplete_details": result.incomplete_details,
                        "usage": result.usage,
                        "raw_response_sha256": sha256_text(result.content),
                    }
                )

            run_record["attempts"].append(rejected_attempt)
            write_json(record_path, run_record)

            print(
                f"[Pln-A] {case_name} / {condition.slug} / "
                f"Run_{repetition_index:03d}, attempt "
                f"{attempt_number}/{max_attempts} rejected: {last_error}",
                file=sys.stderr,
            )
            feedback = last_error

    run_record.update(
        {
            "status": "failed",
            "completed_at_utc": utc_now(),
            "duration_seconds": round(
                time.perf_counter() - run_started,
                6,
            ),
            "failure": {
                "type": "RuntimeError",
                "message": (
                    f"Generation failed after {max_attempts} attempts. "
                    f"Last error: {last_error}"
                ),
            },
        }
    )
    write_json(record_path, run_record)
    print(
        f"[Pln-A] {case_name} / {condition.slug} / "
        f"Run_{repetition_index:03d}: failed",
        file=sys.stderr,
    )
    return run_record


def usage_number(usage: dict[str, Any] | None, *keys: str) -> int:
    """Return the first numeric token field found in an API usage object."""
    if not isinstance(usage, dict):
        return 0
    for key in keys:
        value = usage.get(key)
        if isinstance(value, (int, float)):
            return int(value)
    return 0


def aggregate_record_usage(record: dict[str, Any]) -> dict[str, int]:
    """Aggregate token usage across all agent-level validation attempts."""
    totals = {
        "input_tokens": 0,
        "output_tokens": 0,
        "reasoning_tokens": 0,
        "total_tokens": 0,
    }

    for attempt in record.get("attempts", []):
        if not isinstance(attempt, dict):
            continue
        usage = attempt.get("usage")
        if not isinstance(usage, dict):
            continue

        input_tokens = usage_number(usage, "input_tokens", "prompt_tokens")
        output_tokens = usage_number(
            usage,
            "output_tokens",
            "completion_tokens",
        )
        total_tokens = usage_number(usage, "total_tokens")
        if total_tokens == 0:
            total_tokens = input_tokens + output_tokens

        reasoning_tokens = usage_number(usage, "reasoning_tokens")
        output_details = usage.get("output_tokens_details")
        if isinstance(output_details, dict):
            reasoning_tokens = usage_number(
                output_details,
                "reasoning_tokens",
            ) or reasoning_tokens

        totals["input_tokens"] += input_tokens
        totals["output_tokens"] += output_tokens
        totals["reasoning_tokens"] += reasoning_tokens
        totals["total_tokens"] += total_tokens

    return totals


def record_summary_entry(
    *,
    record: dict[str, Any],
    case_name: str,
    condition: ExperimentCondition,
    repetition_index: int,
    execution_order_index: int,
    output_root: Path,
    source: str,
) -> dict[str, Any]:
    run_dir = (
        output_root
        / case_name
        / condition.slug
        / f"Run_{repetition_index:03d}"
    )
    usage = aggregate_record_usage(record)
    final_validation = record.get("final_validation", {})
    if not isinstance(final_validation, dict):
        final_validation = {}

    unique_functions = final_validation.get("unique_function_calls", [])
    if not isinstance(unique_functions, list):
        unique_functions = []

    return {
        "execution_order_index": execution_order_index,
        "repetition_index": repetition_index,
        "case": case_name,
        "condition": condition.slug,
        "condition_label": condition.label,
        "status": record.get("status"),
        "source": source,
        "duration_seconds": record.get("duration_seconds"),
        "agent_api_calls": len(record.get("attempts", [])),
        "generated_hlaf_call_count": final_validation.get("call_count"),
        "unique_hlaf_function_count": len(unique_functions),
        **usage,
        "output_directory": str(run_dir.relative_to(output_root)),
    }


def write_run_index_csv(path: Path, runs: list[dict[str, Any]]) -> None:
    """Write a flat experiment index for later scoring and cost analysis."""
    fieldnames = [
        "execution_order_index",
        "repetition_index",
        "case",
        "condition",
        "condition_label",
        "status",
        "source",
        "duration_seconds",
        "agent_api_calls",
        "generated_hlaf_call_count",
        "unique_hlaf_function_count",
        "input_tokens",
        "output_tokens",
        "reasoning_tokens",
        "total_tokens",
        "output_directory",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for row in sorted(
            runs,
            key=lambda item: int(item["execution_order_index"]),
        ):
            writer.writerow({key: row.get(key) for key in fieldnames})


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the repeated two-condition Pln-A equivalence experiment "
            "across three wall-panel complexity levels."
        )
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_FILE),
        help="Configuration JSON. Default: config.json beside the script.",
    )
    parser.add_argument(
        "--model",
        default=None,
        help=(
            "Optional model identifier override. OPENAI_MODEL takes precedence "
            "over the active profile when this argument is omitted."
        ),
    )
    parser.add_argument(
        "--crews-file",
        default=str(DEFAULT_CREWS_FILE),
        help="Ext-A-II component-to-crew mapping JSON.",
    )
    parser.add_argument(
        "--functions-file",
        default=str(DEFAULT_FUNCTIONS_FILE),
        help="Headers-and-docstrings-only HLAF Python file.",
    )
    parser.add_argument(
        "--framing-order-file",
        default=str(DEFAULT_FRAMING_ORDER_FILE),
        help="Framing-order rules containing explicit action sequences.",
    )
    parser.add_argument(
        "--sample-elements-file",
        default=str(DEFAULT_SAMPLE_ELEMENTS_FILE),
        help="Worked-sample assembly-instance JSON.",
    )
    parser.add_argument(
        "--sample-sequence-file",
        default=str(DEFAULT_SAMPLE_SEQUENCE_FILE),
        help="Worked-sample HLAF sequence text file.",
    )
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Experiment output directory.",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=30,
        help=(
            "Independent runs for each condition-case cell. Default: 30, "
            "which produces 180 planned runs."
        ),
    )
    parser.add_argument(
        "--parallel-workers",
        type=int,
        default=6,
        help=(
            "Maximum simultaneous Pln-A runs within each repetition batch. "
            "Default: 6, one worker for each case-condition combination. "
            "Use 1 for sequential execution."
        ),
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=None,
        help=(
            "Optional validation-attempt override. By default, use "
            "agent.validation_max_attempts from config.json."
        ),
    )
    parser.add_argument(
        "--delay-seconds",
        type=float,
        default=0.0,
        help=(
            "Optional pause between repetition batches to reduce rate-limit "
            "pressure. The six jobs inside a batch remain parallel."
        ),
    )
    parser.add_argument(
        "--max-runs",
        type=int,
        default=None,
        help=(
            "Optional cap on jobs executed in this invocation. Useful for a "
            "small test before launching the complete experiment."
        ),
    )
    parser.add_argument(
        "--no-prompt-files",
        action="store_true",
        help="Do not save Prompt.txt and Raw_Response.txt for each run.",
    )
    parser.add_argument(
        "--overwrite-existing",
        action="store_true",
        help="Delete and rerun even successfully completed run directories.",
    )
    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help="Stop immediately when one experimental run fails.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Create the execution plan and summary without calling the API.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    experiment_started = time.perf_counter()

    if args.repetitions < 1:
        raise ValueError("--repetitions must be at least 1.")
    if args.parallel_workers < 1:
        raise ValueError("--parallel-workers must be at least 1.")
    if args.delay_seconds < 0:
        raise ValueError("--delay-seconds cannot be negative.")
    if args.max_runs is not None and args.max_runs < 1:
        raise ValueError("--max-runs must be at least 1 when supplied.")

    config_file = resolve_path(args.config)
    crews_file = resolve_path(args.crews_file)
    functions_file = resolve_path(args.functions_file)
    framing_order_file = resolve_path(args.framing_order_file)
    sample_elements_file = resolve_path(args.sample_elements_file)
    sample_sequence_file = resolve_path(args.sample_sequence_file)
    output_root = resolve_path(args.output_root)
    tasks_file = find_tasks_file()

    validate_required_files(
        config_file=config_file,
        crews_file=crews_file,
        tasks_file=tasks_file,
        functions_file=functions_file,
        framing_order_file=framing_order_file,
        sample_elements_file=sample_elements_file,
        sample_sequence_file=sample_sequence_file,
    )

    runtime_config = load_runtime_config(
        config_file,
        model_override=args.model,
    )
    max_attempts = (
        args.max_attempts
        if args.max_attempts is not None
        else runtime_config.validation_max_attempts
    )
    if max_attempts < 1:
        raise ValueError("--max-attempts must be at least 1.")

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key and not args.dry_run:
        raise RuntimeError("OPENAI_API_KEY is not set.")

    crews = normalize_crews(read_json(crews_file))
    tasks = normalize_tasks(read_json(tasks_file))
    design_rules = read_text(framing_order_file)
    sample_assembly = read_json(sample_elements_file)
    sample_sequence = read_text(sample_sequence_file)
    hlaf_catalog, function_specs = extract_hlaf_catalog(functions_file)

    output_root.mkdir(parents=True, exist_ok=True)

    # Each worker thread gets its own reusable OpenAI client. This avoids
    # sharing one SDK client across threads while retaining HTTP connections
    # between repetition batches.
    thread_local = threading.local()

    def get_thread_agent() -> AI_Agent:
        agent = getattr(thread_local, "agent", None)
        if agent is None:
            agent = AI_Agent(
                api_key=str(api_key),
                model=runtime_config.model,
                reasoning_effort=runtime_config.reasoning_effort,
                max_output_tokens=runtime_config.max_output_tokens,
                base_url=runtime_config.base_url,
                timeout_seconds=runtime_config.timeout_seconds,
                sdk_max_retries=runtime_config.sdk_max_retries,
                store=runtime_config.store,
            )
            thread_local.agent = agent
        return agent

    # Keep each repetition as one six-job batch:
    # 3 complexity levels x 2 conditions = 6 concurrent calls.
    batches: list[list[tuple[int, int, str, Path, ExperimentCondition]]] = []
    execution_order_index = 0
    for repetition_index in range(1, args.repetitions + 1):
        batch: list[tuple[int, int, str, Path, ExperimentCondition]] = []
        for case_name, assembly_path in EXPERIMENT_CASES:
            for condition in EXPERIMENT_CONDITIONS:
                execution_order_index += 1
                batch.append(
                    (
                        execution_order_index,
                        repetition_index,
                        case_name,
                        assembly_path,
                        condition,
                    )
                )
        batches.append(batch)

    jobs = [job for batch in batches for job in batch]

    execution_plan = [
        {
            "execution_order_index": order_index,
            "parallel_batch_index": repetition_index,
            "repetition_index": repetition_index,
            "case": case_name,
            "assembly_file": str(assembly_path.relative_to(BASE_DIR)),
            "condition": condition.slug,
            "condition_label": condition.label,
        }
        for (
            order_index,
            repetition_index,
            case_name,
            assembly_path,
            condition,
        ) in jobs
    ]
    write_json(output_root / "Execution_Plan.json", execution_plan)

    summary: dict[str, Any] = {
        "agent": "Pln-A",
        "experiment": (
            "Repeated equivalence evaluation across three wall-panel complexity "
            "levels under (1) framing order plus HLAFs plus human knowledge plus "
            "design information and (2) HLAFs plus design information plus a "
            "worked sample"
        ),
        "status": "planned" if args.dry_run else "running",
        "started_at_utc": utc_now(),
        "completed_at_utc": None,
        "duration_seconds": None,
        "repetitions_per_condition_case": args.repetitions,
        "total_planned_runs": len(jobs),
        "execution_mode": "parallel_by_repetition",
        "parallel_workers": min(args.parallel_workers, 6),
        "jobs_per_repetition_batch": 6,
        "execution_randomized": False,
        "random_seed": None,
        "model_configuration": {
            "profile": runtime_config.profile_name,
            "model": runtime_config.model,
            "api_family": runtime_config.api_family,
            "api_version": runtime_config.api_version,
            "base_url": runtime_config.base_url,
            "reasoning_effort": runtime_config.reasoning_effort,
            "max_output_tokens": runtime_config.max_output_tokens,
            "temperature": None,
            "top_p": None,
            "timeout_seconds": runtime_config.timeout_seconds,
            "sdk_max_retries": runtime_config.sdk_max_retries,
            "store": runtime_config.store,
            "validation_max_attempts": max_attempts,
        },
        "conditions": {
            condition.slug: {
                "label": condition.label,
                "description": condition.description,
                "human_knowledge_included": condition.include_human_knowledge,
                "design_rules_included": condition.include_design_rules,
                "sample_included": condition.include_sample,
            }
            for condition in EXPERIMENT_CONDITIONS
        },
        "cases": [case_name for case_name, _ in EXPERIMENT_CASES],
        "successful_runs": 0,
        "failed_runs": 0,
        "skipped_existing_successes": 0,
        "runs_processed_this_invocation": 0,
        "runs": [],
        "shared_input_files_sha256": {
            config_file.name: sha256_file(config_file),
            functions_file.name: sha256_file(functions_file),
            framing_order_file.name: sha256_file(framing_order_file),
            crews_file.name: sha256_file(crews_file),
            tasks_file.name: sha256_file(tasks_file),
            f"Sample/{sample_elements_file.name}": sha256_file(
                sample_elements_file
            ),
            f"Sample/{sample_sequence_file.name}": sha256_file(
                sample_sequence_file
            ),
        },
        "hlaf_function_count": len(function_specs),
    }

    summary_path = output_root / "Experiment_Summary.json"
    index_path = output_root / "Experiment_Run_Index.csv"
    write_json(summary_path, summary)
    write_run_index_csv(index_path, summary["runs"])

    if args.dry_run:
        summary.update(
            {
                "status": "dry_run_complete",
                "completed_at_utc": utc_now(),
                "duration_seconds": round(
                    time.perf_counter() - experiment_started,
                    6,
                ),
            }
        )
        write_json(summary_path, summary)
        print(f"Generated execution plan: {output_root / 'Execution_Plan.json'}")
        print(f"Planned runs: {len(jobs)}")
        return

    any_failure = False
    jobs_executed_now = 0
    stop_after_batch = False

    def inspect_existing_run(
        *,
        case_name: str,
        condition: ExperimentCondition,
        repetition_index: int,
    ) -> tuple[Path, dict[str, Any] | None]:
        run_dir = (
            output_root
            / case_name
            / condition.slug
            / f"Run_{repetition_index:03d}"
        )
        record_path = run_dir / "Run_Record.json"
        sequence_path = run_dir / "Final_Function_Sequence.py"

        existing_record: dict[str, Any] | None = None
        if record_path.exists() and sequence_path.exists():
            try:
                candidate = read_json(record_path)
                if isinstance(candidate, dict):
                    existing_record = candidate
            except (OSError, json.JSONDecodeError):
                existing_record = None
        return run_dir, existing_record

    def execute_job(
        job: tuple[int, int, str, Path, ExperimentCondition],
    ) -> tuple[tuple[int, int, str, Path, ExperimentCondition], dict[str, Any]]:
        (
            order_index,
            repetition_index,
            case_name,
            assembly_path,
            condition,
        ) = job
        record = execute_single_run(
            agent=get_thread_agent(),
            runtime_config=runtime_config,
            case_name=case_name,
            assembly_path=assembly_path,
            condition=condition,
            repetition_index=repetition_index,
            execution_order_index=order_index,
            output_root=output_root,
            design_rules=design_rules,
            hlaf_catalog=hlaf_catalog,
            function_specs=function_specs,
            crews=crews,
            tasks=tasks,
            sample_assembly=sample_assembly,
            sample_sequence=sample_sequence,
            tasks_file=tasks_file,
            crews_file=crews_file,
            functions_file=functions_file,
            framing_order_file=framing_order_file,
            sample_elements_file=sample_elements_file,
            sample_sequence_file=sample_sequence_file,
            max_attempts=max_attempts,
            save_prompt=not args.no_prompt_files,
        )
        return job, record

    worker_count = min(args.parallel_workers, 6)
    with ThreadPoolExecutor(
        max_workers=worker_count,
        thread_name_prefix="pln-a",
    ) as executor:
        for batch_number, batch in enumerate(batches, start=1):
            if stop_after_batch:
                break

            futures: dict[
                Future[tuple[tuple[int, int, str, Path, ExperimentCondition], dict[str, Any]]],
                tuple[int, int, str, Path, ExperimentCondition],
            ] = {}

            print(
                f"[Batch {batch_number:03d}/{len(batches):03d}] "
                f"preparing {len(batch)} case-condition combinations "
                f"with up to {worker_count} parallel workers."
            )

            for job in batch:
                (
                    order_index,
                    repetition_index,
                    case_name,
                    assembly_path,
                    condition,
                ) = job

                run_dir, existing_record = inspect_existing_run(
                    case_name=case_name,
                    condition=condition,
                    repetition_index=repetition_index,
                )

                if (
                    existing_record is not None
                    and existing_record.get("status") == "success"
                    and not args.overwrite_existing
                ):
                    entry = record_summary_entry(
                        record=existing_record,
                        case_name=case_name,
                        condition=condition,
                        repetition_index=repetition_index,
                        execution_order_index=order_index,
                        output_root=output_root,
                        source="existing_success",
                    )
                    summary["runs"].append(entry)
                    summary["successful_runs"] += 1
                    summary["skipped_existing_successes"] += 1
                    write_json(summary_path, summary)
                    write_run_index_csv(index_path, summary["runs"])
                    print(
                        f"[Skip] {case_name} / {condition.slug} / "
                        f"Run_{repetition_index:03d} already succeeded."
                    )
                    continue

                if args.max_runs is not None and jobs_executed_now >= args.max_runs:
                    stop_after_batch = True
                    break

                if run_dir.exists():
                    shutil.rmtree(run_dir)

                future = executor.submit(execute_job, job)
                futures[future] = job
                jobs_executed_now += 1
                summary["runs_processed_this_invocation"] = jobs_executed_now

            # Only the main thread updates the shared summary/index files.
            for future in as_completed(futures):
                job = futures[future]
                (
                    order_index,
                    repetition_index,
                    case_name,
                    _assembly_path,
                    condition,
                ) = job

                try:
                    _returned_job, record = future.result()
                except Exception as exc:
                    any_failure = True
                    record = {
                        "status": "failed",
                        "duration_seconds": None,
                        "attempts": [],
                        "failure": {
                            "type": type(exc).__name__,
                            "message": str(exc),
                        },
                    }
                    print(
                        f"[Pln-A] {case_name} / {condition.slug} / "
                        f"Run_{repetition_index:03d}: worker failure: "
                        f"{type(exc).__name__}: {exc}",
                        file=sys.stderr,
                    )

                entry = record_summary_entry(
                    record=record,
                    case_name=case_name,
                    condition=condition,
                    repetition_index=repetition_index,
                    execution_order_index=order_index,
                    output_root=output_root,
                    source="executed_this_invocation",
                )
                summary["runs"].append(entry)

                if record.get("status") == "success":
                    summary["successful_runs"] += 1
                else:
                    summary["failed_runs"] += 1
                    any_failure = True

                write_json(summary_path, summary)
                write_run_index_csv(index_path, summary["runs"])

            if any_failure and args.stop_on_error:
                # Already-submitted jobs in the current batch are allowed to finish.
                stop_after_batch = True

            if args.delay_seconds > 0 and batch_number < len(batches):
                time.sleep(args.delay_seconds)

    processed_or_existing = (
        summary["successful_runs"] + summary["failed_runs"]
    )
    summary.update(
        {
            "status": (
                "completed_with_failures"
                if any_failure
                else (
                    "partial_invocation_complete"
                    if processed_or_existing < len(jobs)
                    else "success"
                )
            ),
            "completed_at_utc": utc_now(),
            "duration_seconds": round(
                time.perf_counter() - experiment_started,
                6,
            ),
        }
    )
    write_json(summary_path, summary)
    write_run_index_csv(index_path, summary["runs"])

    print(f"Generated: {summary_path}")
    print(f"Generated: {index_path}")
    print(
        "Successful/available runs: "
        f"{summary['successful_runs']}/{summary['total_planned_runs']}"
    )
    if summary["failed_runs"]:
        print(f"Failed runs: {summary['failed_runs']}", file=sys.stderr)

    if any_failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
