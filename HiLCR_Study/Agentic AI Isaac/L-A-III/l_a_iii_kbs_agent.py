#!/usr/bin/env python3
"""
L-A-III for the finalized KBS HLAF library.

This AI agent reads a Python file containing finalized top-level HLAF
functions, asks the configured OpenAI model (for example, o3) to create a
descriptive docstring for each function, and writes a codified copy.

The executable statements are preserved. Only function docstrings may change.

Typical directory:
    l_a_iii_kbs_agent.py
    KBS_Simplified.py
    config.json

The source may also be named KBS_Simplified(1).py. Use --source to select an
exact file when multiple candidates exist.

Outputs:
    KBS_Simplified_Codified.py
    L_A_III_KBS_Run_Record.json

Required for generation:
    OPENAI_API_KEY

Optional:
    OPENAI_BASE_URL
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.metadata
import json
import os
import platform
import sys
import textwrap
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

try:
    from openai import AuthenticationError, BadRequestError, OpenAI
except ModuleNotFoundError:
    class _MissingOpenAIError(Exception):
        pass

    AuthenticationError = _MissingOpenAIError
    BadRequestError = _MissingOpenAIError
    OpenAI = None


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT_NAME = "KBS_Simplified_Codified.py"
DEFAULT_RECORD_NAME = "L_A_III_KBS_Run_Record.json"
CONFIG_NAME = "config.json"

TOOL_NAME = "submit_l_a_iii_docstring"

VARIABLE_CONTEXT: dict[str, str] = {
    "el_name": (
        "Name of the element object used by the simulation and attachment or "
        "detachment operations."
    ),
    "L": "Element length, expressed in the simulation's metric units.",
    "W": "Element width, expressed in the simulation's metric units.",
    "H": "Element height, expressed in the simulation's metric units.",
    "X": "Element or target x-coordinate used by the procedure.",
    "Y": "Element or target y-coordinate used by the procedure.",
    "Z": "Element or target z-coordinate used by the procedure.",
    "push_to_nail": (
        "Linear push offset used to advance the nail-gun TCP during a nailing "
        "motion."
    ),
    "Side_Selector": (
        "Conveyor-side indicator used by the procedure; the implemented code "
        "typically treats +1 and -1 as the valid sides."
    ),
    "If_Tangent_From_Left": (
        "Whether the element is tangent to its neighboring element from the "
        "left side."
    ),
    "Is_Held": (
        "Whether the element remains held during the nailing or release "
        "procedure."
    ),
    "el_pose": (
        "Element pose data consumed by the function when calculating robot "
        "targets."
    ),
    "el_dims": (
        "Element dimension data consumed by the function when calculating "
        "robot targets."
    ),
    "conv_current_location": (
        "Current smart-conveyor joint location used to calculate subsequent "
        "conveyor and robot targets."
    ),
    "nail_num": "Number of nailing operations requested by the procedure.",
    "Sht_plate_on_conv_loc": (
        "Recorded conveyor location associated with the sheathing plate."
    ),
}

DOCSTRING_TOOL: dict[str, Any] = {
    "type": "function",
    "name": TOOL_NAME,
    "description": (
        "Submit structured documentation for exactly one finalized HLAF "
        "function."
    ),
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "function_name": {
                "type": "string",
                "description": "Exact supplied Python function name.",
            },
            "summary": {
                "type": "string",
                "description": (
                    "Concise description of the physical robotic or station "
                    "operation performed by the function."
                ),
            },
            "procedure_steps": {
                "type": "array",
                "description": (
                    "Ordered descriptions of the function's implemented "
                    "workflow."
                ),
                "items": {"type": "string"},
            },
            "arguments": {
                "type": "array",
                "description": (
                    "One description for every declared function argument."
                ),
                "items": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "description": {"type": "string"},
                    },
                    "required": ["name", "description"],
                    "additionalProperties": False,
                },
            },
            "returns": {
                "type": "string",
                "description": (
                    "Grounded description of the actual return behavior."
                ),
            },
            "operational_notes": {
                "type": "array",
                "description": (
                    "Important conditions, reachability branches, side "
                    "selection, synchronization, or simulation side effects. "
                    "Use an empty array when none are needed."
                ),
                "items": {"type": "string"},
            },
        },
        "required": [
            "function_name",
            "summary",
            "procedure_steps",
            "arguments",
            "returns",
            "operational_notes",
        ],
        "additionalProperties": False,
    },
}

SYSTEM_INSTRUCTIONS = """
You are L-A-III, the HLAF Codification Agent.

You receive one finalized Python function from a robotic construction
knowledge-base library. Generate accurate descriptive documentation for that
function.

Requirements:
- Copy the function name and all argument names exactly.
- Describe the implemented physical operation, not merely Python syntax.
- Explain the execution in its actual order.
- Ground every statement in the supplied function, its signature, comments,
  static analysis, and known argument semantics.
- Describe every declared argument exactly once.
- Accurately describe explicit return values and conditional return behavior.
- Mention robot identities, TCP/tool identifiers, conveyor motions, attachment,
  detachment, gravity changes, reachability branches, or home motions only when
  they are present.
- Preserve the distinction between planning/executing a nail-gun trajectory and
  physically firing a nail. Do not claim a firing command exists unless it is
  explicitly present.
- Do not claim that the function performs an LLM call, recursive pose
  correction, validation, retries, or calculations absent from the code.
- Do not invent units, tolerances, fastener counts, safety guarantees, or
  success guarantees.
- Do not rewrite or return the Python function.
- Do not return Markdown or triple-quoted text.
- Return documentation only through submit_l_a_iii_docstring.
""".strip()


@dataclass(frozen=True)
class ParameterInfo:
    name: str
    annotation: str | None
    default: str | None
    known_semantics: str


@dataclass(frozen=True)
class FunctionAnalysis:
    function_name: str
    parameters: tuple[ParameterInfo, ...]
    return_annotation: str | None
    explicit_returns: tuple[str, ...]
    robot_names: tuple[str, ...]
    tcp_names: tuple[str, ...]
    called_attributes: tuple[str, ...]
    called_functions: tuple[str, ...]
    branch_count: int
    loop_count: int
    existing_docstring: str | None


@dataclass(frozen=True)
class FunctionInfo:
    node: ast.FunctionDef | ast.AsyncFunctionDef
    source: str
    analysis: FunctionAnalysis


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def write_json(path: Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(value, file, indent=2, ensure_ascii=False)
        file.write("\n")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def clean_text(value: str) -> str:
    return " ".join(value.strip().split()).replace('"""', r'\"\"\"')


def usage_to_dict(response: Any) -> dict[str, Any] | None:
    usage = getattr(response, "usage", None)
    if hasattr(usage, "model_dump"):
        return usage.model_dump(mode="json")
    return None


def source_segment(
    source: str,
    node: ast.AST | None,
) -> str | None:
    if node is None:
        return None

    segment = ast.get_source_segment(source, node)
    if segment is not None:
        return segment.strip()

    try:
        return ast.unparse(node).strip()
    except Exception:
        return None


def discover_source_file(
    requested_source: str | None,
    output_path: Path,
) -> Path:
    if requested_source:
        candidate = Path(requested_source).expanduser()
        if not candidate.is_absolute():
            candidate = ROOT / candidate
        if not candidate.exists():
            raise FileNotFoundError(
                f"Source file does not exist: {candidate}"
            )
        return candidate.resolve()

    preferred = [
        ROOT / "KBS_Simplified.py",
        ROOT / "KBS_Simplified(1).py",
    ]

    for candidate in preferred:
        if candidate.exists() and candidate.resolve() != output_path.resolve():
            return candidate.resolve()

    candidates = sorted(
        path.resolve()
        for path in ROOT.glob("KBS_Simplified*.py")
        if path.resolve() != output_path.resolve()
        and "Codified" not in path.name
    )

    if not candidates:
        raise FileNotFoundError(
            "No KBS_Simplified*.py source file was found. "
            "Use --source to provide its path."
        )

    if len(candidates) > 1:
        names = ", ".join(path.name for path in candidates)
        raise RuntimeError(
            "Multiple KBS source files were found: "
            f"{names}. Use --source to select one."
        )

    return candidates[0]


def get_model_profile(config: dict[str, Any]) -> dict[str, Any]:
    profile_name = config["active_model_profile"]
    return config["model_profiles"][profile_name]


def create_client(config: dict[str, Any]) -> Any:
    if OpenAI is None:
        raise RuntimeError(
            "The openai package is not installed. Run: "
            "pip install -r requirements_l_a_iii_kbs.txt"
        )

    api = config.get("api", {})

    options: dict[str, Any] = {
        "api_key": os.environ["OPENAI_API_KEY"],
        "timeout": api.get("timeout_seconds", 240),
        "max_retries": api.get("sdk_max_retries", 2),
    }

    base_url = os.getenv("OPENAI_BASE_URL") or api.get("base_url")
    if base_url:
        options["base_url"] = base_url

    return OpenAI(**options)


def request_parameters(
    config: dict[str, Any],
    prompt: str,
    force_tool: bool,
) -> dict[str, Any]:
    model = get_model_profile(config)
    stage = config.get("l_a_iii", {})

    request: dict[str, Any] = {
        "model": model["identifier"],
        "instructions": SYSTEM_INSTRUCTIONS,
        "input": prompt,
        "tools": [DOCSTRING_TOOL],
        "tool_choice": (
            {"type": "function", "name": TOOL_NAME}
            if force_tool
            else "auto"
        ),
        "parallel_tool_calls": False,
        "max_output_tokens": stage.get(
            "max_output_tokens",
            model.get("max_output_tokens", 4096),
        ),
        "store": config.get("request", {}).get("store", False),
    }

    if model.get("reasoning_effort") is not None:
        request["reasoning"] = {
            "effort": model["reasoning_effort"]
        }

    if model.get("temperature") is not None:
        request["temperature"] = model["temperature"]

    if model.get("top_p") is not None:
        request["top_p"] = model["top_p"]

    return request


def collect_parameters(
    source: str,
    node: ast.FunctionDef | ast.AsyncFunctionDef,
) -> tuple[ParameterInfo, ...]:
    positional = [
        *node.args.posonlyargs,
        *node.args.args,
    ]
    positional_defaults: dict[str, ast.expr] = {}

    if node.args.defaults:
        defaulted = positional[-len(node.args.defaults):]
        positional_defaults = {
            argument.arg: default
            for argument, default in zip(
                defaulted,
                node.args.defaults,
            )
        }

    parameters: list[ParameterInfo] = []

    for argument in positional:
        parameters.append(
            ParameterInfo(
                name=argument.arg,
                annotation=source_segment(
                    source,
                    argument.annotation,
                ),
                default=source_segment(
                    source,
                    positional_defaults.get(argument.arg),
                ),
                known_semantics=VARIABLE_CONTEXT.get(
                    argument.arg,
                    (
                        "Infer this argument's role strictly from the "
                        "function body and signature."
                    ),
                ),
            )
        )

    if node.args.vararg is not None:
        argument = node.args.vararg
        parameters.append(
            ParameterInfo(
                name=argument.arg,
                annotation=source_segment(
                    source,
                    argument.annotation,
                ),
                default="*args",
                known_semantics=(
                    "Variadic positional arguments; describe only from "
                    "their implemented use."
                ),
            )
        )

    for argument, default in zip(
        node.args.kwonlyargs,
        node.args.kw_defaults,
    ):
        parameters.append(
            ParameterInfo(
                name=argument.arg,
                annotation=source_segment(
                    source,
                    argument.annotation,
                ),
                default=source_segment(source, default),
                known_semantics=VARIABLE_CONTEXT.get(
                    argument.arg,
                    (
                        "Infer this argument's role strictly from the "
                        "function body and signature."
                    ),
                ),
            )
        )

    if node.args.kwarg is not None:
        argument = node.args.kwarg
        parameters.append(
            ParameterInfo(
                name=argument.arg,
                annotation=source_segment(
                    source,
                    argument.annotation,
                ),
                default="**kwargs",
                known_semantics=(
                    "Variadic keyword arguments; describe only from "
                    "their implemented use."
                ),
            )
        )

    return tuple(parameters)


def analyze_function(
    source: str,
    node: ast.FunctionDef | ast.AsyncFunctionDef,
) -> FunctionAnalysis:
    robots: set[str] = set()
    tcp_names: set[str] = set()
    called_attributes: set[str] = set()
    called_functions: set[str] = set()
    explicit_returns: list[str] = []
    branch_count = 0
    loop_count = 0

    for child in ast.walk(node):
        if isinstance(child, ast.Name):
            if child.id.startswith("Robot_"):
                robots.add(child.id)

        if isinstance(child, ast.Call):
            if isinstance(child.func, ast.Attribute):
                called_attributes.add(child.func.attr)
            elif isinstance(child.func, ast.Name):
                called_functions.add(child.func.id)

            for keyword_argument in child.keywords:
                if (
                    keyword_argument.arg == "tcp_name"
                    and isinstance(
                        keyword_argument.value,
                        ast.Constant,
                    )
                    and isinstance(
                        keyword_argument.value.value,
                        str,
                    )
                ):
                    tcp_names.add(
                        keyword_argument.value.value
                    )

        if isinstance(child, ast.Return):
            if child.value is None:
                explicit_returns.append("None")
            else:
                explicit_returns.append(
                    source_segment(source, child.value)
                    or ast.dump(child.value)
                )

        if isinstance(child, (ast.If, ast.IfExp, ast.Match)):
            branch_count += 1

        if isinstance(
            child,
            (ast.For, ast.AsyncFor, ast.While),
        ):
            loop_count += 1

    if not explicit_returns:
        explicit_returns.append(
            "Implicit None at normal function completion"
        )

    return FunctionAnalysis(
        function_name=node.name,
        parameters=collect_parameters(source, node),
        return_annotation=source_segment(
            source,
            node.returns,
        ),
        explicit_returns=tuple(
            dict.fromkeys(explicit_returns)
        ),
        robot_names=tuple(sorted(robots)),
        tcp_names=tuple(sorted(tcp_names)),
        called_attributes=tuple(
            sorted(called_attributes)
        ),
        called_functions=tuple(
            sorted(called_functions)
        ),
        branch_count=branch_count,
        loop_count=loop_count,
        existing_docstring=ast.get_docstring(
            node,
            clean=False,
        ),
    )


def extract_top_level_functions(
    source: str,
    source_path: Path,
) -> tuple[ast.Module, list[FunctionInfo]]:
    tree = ast.parse(source, filename=str(source_path))
    functions: list[FunctionInfo] = []

    for node in tree.body:
        if not isinstance(
            node,
            (ast.FunctionDef, ast.AsyncFunctionDef),
        ):
            continue

        segment = ast.get_source_segment(source, node)
        if segment is None:
            raise ValueError(
                f"Could not extract source for {node.name!r}."
            )

        functions.append(
            FunctionInfo(
                node=node,
                source=segment,
                analysis=analyze_function(
                    source,
                    node,
                ),
            )
        )

    if not functions:
        raise ValueError(
            "No top-level Python functions were found."
        )

    return tree, functions


def build_prompt(function: FunctionInfo) -> str:
    analysis = function.analysis

    static_context = {
        "function_name": analysis.function_name,
        "parameters": [
            asdict(parameter)
            for parameter in analysis.parameters
        ],
        "return_annotation": analysis.return_annotation,
        "observed_return_expressions": list(
            analysis.explicit_returns
        ),
        "robots_referenced": list(
            analysis.robot_names
        ),
        "tcp_names_referenced": list(
            analysis.tcp_names
        ),
        "called_methods": list(
            analysis.called_attributes
        ),
        "called_functions": list(
            analysis.called_functions
        ),
        "conditional_branch_count": (
            analysis.branch_count
        ),
        "loop_count": analysis.loop_count,
        "existing_docstring": (
            analysis.existing_docstring
        ),
    }

    return (
        "Create the finalized descriptive documentation for this "
        "HLAF function.\n\n"
        "Static function context:\n"
        + json.dumps(
            static_context,
            indent=2,
            ensure_ascii=False,
        )
        + "\n\nFinalized Python function:\n```python\n"
        + function.source
        + "\n```"
    )


def extract_tool_arguments(response: Any) -> dict[str, Any]:
    for item in response.output:
        if (
            item.type == "function_call"
            and item.name == TOOL_NAME
        ):
            return json.loads(item.arguments)

    raise RuntimeError(
        f"The model did not call {TOOL_NAME}."
    )


def validate_documentation(
    function: FunctionInfo,
    arguments: dict[str, Any],
) -> dict[str, Any]:
    expected_name = function.analysis.function_name

    if arguments.get("function_name") != expected_name:
        raise ValueError(
            "Returned function name does not match "
            f"{expected_name!r}."
        )

    summary = clean_text(arguments.get("summary", ""))
    if not summary:
        raise ValueError("Summary is empty.")

    procedure_steps = [
        clean_text(step)
        for step in arguments.get(
            "procedure_steps",
            [],
        )
        if isinstance(step, str) and step.strip()
    ]
    if not procedure_steps:
        raise ValueError(
            "At least one procedure step is required."
        )

    expected_arguments = [
        parameter.name
        for parameter in function.analysis.parameters
    ]

    descriptions: dict[str, str] = {}

    for item in arguments.get("arguments", []):
        name = item.get("name")
        description = clean_text(
            item.get("description", "")
        )

        if name in descriptions:
            raise ValueError(
                f"Argument {name!r} was returned twice."
            )
        if not description:
            raise ValueError(
                f"Argument {name!r} has no description."
            )

        descriptions[name] = description

    if set(descriptions) != set(expected_arguments):
        missing = sorted(
            set(expected_arguments) - set(descriptions)
        )
        extra = sorted(
            set(descriptions) - set(expected_arguments)
        )
        raise ValueError(
            "Argument documentation mismatch. "
            f"Missing={missing}; extra={extra}"
        )

    returns = clean_text(arguments.get("returns", ""))
    if not returns:
        raise ValueError(
            "Return behavior is not described."
        )

    operational_notes = [
        clean_text(note)
        for note in arguments.get(
            "operational_notes",
            [],
        )
        if isinstance(note, str) and note.strip()
    ]

    return {
        "function_name": expected_name,
        "summary": summary,
        "procedure_steps": procedure_steps,
        "arguments": [
            {
                "name": name,
                "description": descriptions[name],
            }
            for name in expected_arguments
        ],
        "returns": returns,
        "operational_notes": operational_notes,
    }


def generate_documentation(
    client: OpenAI,
    config: dict[str, Any],
    function: FunctionInfo,
) -> tuple[dict[str, Any], list[dict[str, Any]], str]:
    stage = config.get("l_a_iii", {})
    max_attempts = stage.get(
        "validation_max_attempts",
        config.get("agent", {}).get(
            "validation_max_attempts",
            3,
        ),
    )
    fallback_to_auto = stage.get(
        "fallback_to_auto_tool_choice",
        True,
    )

    prompt = build_prompt(function)
    feedback = ""
    attempts: list[dict[str, Any]] = []
    last_error = "Unknown documentation failure."

    for attempt_number in range(1, max_attempts + 1):
        started = time.perf_counter()
        response = None
        force_tool = True

        try:
            request_prompt = prompt + feedback

            try:
                response = client.responses.create(
                    **request_parameters(
                        config,
                        request_prompt,
                        force_tool=True,
                    )
                )
            except BadRequestError as exc:
                if (
                    fallback_to_auto
                    and "tool_choice"
                    in str(exc).casefold()
                ):
                    force_tool = False
                    response = client.responses.create(
                        **request_parameters(
                            config,
                            request_prompt
                            + "\n\nYou must call "
                            + TOOL_NAME
                            + " in this response.",
                            force_tool=False,
                        )
                    )
                else:
                    raise

            if getattr(response, "status", None) == "incomplete":
                details = getattr(
                    response,
                    "incomplete_details",
                    None,
                )
                if hasattr(details, "model_dump"):
                    details = details.model_dump(mode="json")
                raise RuntimeError(
                    f"Incomplete response: {details}"
                )

            raw = extract_tool_arguments(response)
            validated = validate_documentation(
                function,
                raw,
            )

            attempts.append(
                {
                    "attempt": attempt_number,
                    "status": "accepted",
                    "tool_choice_mode": (
                        "forced_function"
                        if force_tool
                        else "auto_fallback"
                    ),
                    "response_id": getattr(
                        response,
                        "id",
                        None,
                    ),
                    "returned_model_identifier": getattr(
                        response,
                        "model",
                        None,
                    ),
                    "response_status": getattr(
                        response,
                        "status",
                        None,
                    ),
                    "duration_seconds": round(
                        time.perf_counter() - started,
                        6,
                    ),
                    "usage": usage_to_dict(response),
                    "tool_arguments": raw,
                    "validated_documentation": validated,
                }
            )

            return validated, attempts, prompt

        except AuthenticationError:
            raise

        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"

            attempt_record: dict[str, Any] = {
                "attempt": attempt_number,
                "status": "rejected",
                "duration_seconds": round(
                    time.perf_counter() - started,
                    6,
                ),
                "error": last_error,
            }

            if response is not None:
                attempt_record.update(
                    {
                        "response_id": getattr(
                            response,
                            "id",
                            None,
                        ),
                        "returned_model_identifier": getattr(
                            response,
                            "model",
                            None,
                        ),
                        "response_status": getattr(
                            response,
                            "status",
                            None,
                        ),
                        "usage": usage_to_dict(response),
                    }
                )

            attempts.append(attempt_record)

            print(
                f"[L-A-III] {function.analysis.function_name}, "
                f"attempt {attempt_number}/{max_attempts} "
                f"rejected: {last_error}",
                file=sys.stderr,
            )

            feedback = (
                "\n\nThe previous documentation was rejected:\n"
                + last_error
                + "\nRegenerate the documentation. Copy the "
                "function name and every argument name exactly."
            )

    raise RuntimeError(
        "L-A-III failed for "
        f"{function.analysis.function_name!r} after "
        f"{max_attempts} attempts. Last error: {last_error}"
    )


def wrap_text(
    text: str,
    *,
    initial_indent: str,
    subsequent_indent: str,
    width: int,
) -> list[str]:
    return textwrap.wrap(
        text,
        width=width,
        initial_indent=initial_indent,
        subsequent_indent=subsequent_indent,
        break_long_words=False,
        break_on_hyphens=False,
    ) or [initial_indent.rstrip()]


def format_docstring(
    function: FunctionInfo,
    documentation: dict[str, Any],
    body_indent: str,
) -> list[str]:
    width = max(52, 92 - len(body_indent))

    lines: list[str] = [
        body_indent + '"""' + documentation["summary"],
        "",
        body_indent + "Procedure:",
    ]

    for index, step in enumerate(
        documentation["procedure_steps"],
        start=1,
    ):
        wrapped = wrap_text(
            step,
            initial_indent=f"    {index}. ",
            subsequent_indent="       ",
            width=width,
        )
        lines.extend(
            body_indent + line
            for line in wrapped
        )

    if documentation["arguments"]:
        lines.extend(["", body_indent + "Args:"])

        parameter_by_name = {
            parameter.name: parameter
            for parameter in function.analysis.parameters
        }

        for item in documentation["arguments"]:
            parameter = parameter_by_name[item["name"]]
            annotation = (
                f" ({parameter.annotation})"
                if parameter.annotation
                else ""
            )
            label = (
                f"    {parameter.name}{annotation}: "
            )
            wrapped = wrap_text(
                item["description"],
                initial_indent=label,
                subsequent_indent=" " * len(label),
                width=width,
            )
            lines.extend(
                body_indent + line
                for line in wrapped
            )

    lines.extend(["", body_indent + "Returns:"])
    lines.extend(
        body_indent + line
        for line in wrap_text(
            documentation["returns"],
            initial_indent="    ",
            subsequent_indent="    ",
            width=width,
        )
    )

    if documentation["operational_notes"]:
        lines.extend(["", body_indent + "Notes:"])

        for note in documentation[
            "operational_notes"
        ]:
            wrapped = wrap_text(
                note,
                initial_indent="    - ",
                subsequent_indent="      ",
                width=width,
            )
            lines.extend(
                body_indent + line
                for line in wrapped
            )

    lines.append(body_indent + '"""')
    return lines


def replace_docstrings(
    original_source: str,
    functions: list[FunctionInfo],
    documentation: dict[str, dict[str, Any]],
) -> str:
    lines = original_source.splitlines()
    replacements: list[
        tuple[int, int, list[str]]
    ] = []

    for function in functions:
        node = function.node
        body_indent = " " * (node.col_offset + 4)
        replacement = format_docstring(
            function,
            documentation[node.name],
            body_indent,
        )

        first_statement = node.body[0] if node.body else None

        has_docstring = (
            isinstance(first_statement, ast.Expr)
            and isinstance(
                first_statement.value,
                ast.Constant,
            )
            and isinstance(
                first_statement.value.value,
                str,
            )
        )

        if has_docstring:
            start = first_statement.lineno - 1
            end = first_statement.end_lineno
        else:
            if first_statement is None:
                raise ValueError(
                    f"Function {node.name!r} has no body."
                )
            start = first_statement.lineno - 1
            end = start

        replacements.append(
            (start, end, replacement)
        )

    for start, end, replacement in sorted(
        replacements,
        key=lambda item: item[0],
        reverse=True,
    ):
        lines[start:end] = replacement

    return "\n".join(lines) + "\n"


class RemoveDocstrings(ast.NodeTransformer):
    def _strip(self, node: Any) -> Any:
        self.generic_visit(node)

        if (
            getattr(node, "body", None)
            and isinstance(node.body[0], ast.Expr)
            and isinstance(
                node.body[0].value,
                ast.Constant,
            )
            and isinstance(
                node.body[0].value.value,
                str,
            )
        ):
            node.body = node.body[1:]

        return node

    def visit_Module(self, node: ast.Module) -> ast.AST:
        return self._strip(node)

    def visit_ClassDef(self, node: ast.ClassDef) -> ast.AST:
        return self._strip(node)

    def visit_FunctionDef(
        self,
        node: ast.FunctionDef,
    ) -> ast.AST:
        return self._strip(node)

    def visit_AsyncFunctionDef(
        self,
        node: ast.AsyncFunctionDef,
    ) -> ast.AST:
        return self._strip(node)


def executable_ast(source: str) -> str:
    tree = ast.parse(source)
    tree = RemoveDocstrings().visit(tree)
    ast.fix_missing_locations(tree)

    return ast.dump(
        tree,
        annotate_fields=True,
        include_attributes=False,
    )


def validate_codified_source(
    original_source: str,
    codified_source: str,
    output_path: Path,
) -> None:
    compile(
        codified_source,
        str(output_path),
        "exec",
    )

    if executable_ast(original_source) != executable_ast(
        codified_source
    ):
        raise ValueError(
            "Executable content changed. L-A-III is permitted "
            "to change only function docstrings."
        )


def reusable_documentation(
    previous_record: dict[str, Any],
    source_hash: str,
    instructions_hash: str,
    function: FunctionInfo,
) -> dict[str, Any] | None:
    if (
        previous_record.get("source_sha256")
        != source_hash
        or previous_record.get(
            "instructions_sha256"
        )
        != instructions_hash
    ):
        return None

    previous = previous_record.get(
        "functions",
        {},
    ).get(function.analysis.function_name)

    if not previous:
        return None

    candidate = previous.get("documentation")
    if not isinstance(candidate, dict):
        return None

    try:
        return validate_documentation(
            function,
            candidate,
        )
    except Exception:
        return None


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Use o3 to add descriptive docstrings to finalized "
            "top-level KBS HLAF functions."
        )
    )
    parser.add_argument(
        "--source",
        help=(
            "Input Python file. Defaults to KBS_Simplified.py or "
            "KBS_Simplified(1).py in the script directory."
        ),
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_NAME,
        help=f"Output filename (default: {DEFAULT_OUTPUT_NAME}).",
    )
    parser.add_argument(
        "--record",
        default=DEFAULT_RECORD_NAME,
        help=f"Run-record filename (default: {DEFAULT_RECORD_NAME}).",
    )
    parser.add_argument(
        "--expected-count",
        type=int,
        default=27,
        help=(
            "Required number of top-level functions. Set to 0 to "
            "disable the count check. Default: 27."
        ),
    )
    parser.add_argument(
        "--inspect",
        action="store_true",
        help=(
            "List detected functions without calling the API or "
            "writing a codified file."
        ),
    )
    return parser.parse_args()


def resolve_local_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def main() -> None:
    args = parse_arguments()
    started = time.perf_counter()

    output_path = resolve_local_path(args.output)
    record_path = resolve_local_path(args.record)
    config_path = ROOT / CONFIG_NAME

    source_path = discover_source_file(
        args.source,
        output_path,
    )
    original_source = source_path.read_text(
        encoding="utf-8"
    )
    _, functions = extract_top_level_functions(
        original_source,
        source_path,
    )

    if (
        args.expected_count > 0
        and len(functions) != args.expected_count
    ):
        raise ValueError(
            f"Expected {args.expected_count} top-level functions, "
            f"but found {len(functions)} in {source_path.name}."
        )

    if args.inspect:
        print(
            f"Detected {len(functions)} top-level functions in "
            f"{source_path.name}:"
        )
        for index, function in enumerate(functions, start=1):
            print(
                f"{index:02d}. "
                f"{function.analysis.function_name}"
            )
        return

    if not os.getenv("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is not set.")

    if not config_path.exists():
        raise FileNotFoundError(
            f"Missing configuration file: {config_path}"
        )

    config = read_json(config_path)
    source_hash = sha256_file(source_path)
    instructions_hash = sha256_text(
        SYSTEM_INSTRUCTIONS
    )

    previous_record: dict[str, Any] = {}
    if record_path.exists():
        try:
            previous_record = read_json(record_path)
        except Exception:
            previous_record = {}

    model = get_model_profile(config)
    api = config.get("api", {})
    stage = config.get("l_a_iii", {})

    record: dict[str, Any] = {
        "agent": "L-A-III",
        "purpose": (
            "Add descriptive docstrings to finalized KBS HLAF "
            "functions."
        ),
        "status": "running",
        "started_at_utc": utc_now(),
        "completed_at_utc": None,
        "duration_seconds": None,
        "files": {
            "source": source_path.name,
            "configuration": config_path.name,
            "output": output_path.name,
            "run_record": record_path.name,
        },
        "source_sha256": source_hash,
        "configuration_sha256": sha256_file(
            config_path
        ),
        "instructions": SYSTEM_INSTRUCTIONS,
        "instructions_sha256": instructions_hash,
        "function_count": len(functions),
        "function_order": [
            function.analysis.function_name
            for function in functions
        ],
        "configuration": {
            "active_model_profile": config[
                "active_model_profile"
            ],
            "requested_model_identifier": model[
                "identifier"
            ],
            "api_family": api.get("api_family"),
            "api_version": api.get("api_version"),
            "base_url": (
                os.getenv("OPENAI_BASE_URL")
                or api.get("base_url")
                or "OpenAI default"
            ),
            "reasoning_effort": model.get(
                "reasoning_effort"
            ),
            "temperature": model.get("temperature"),
            "top_p": model.get("top_p"),
            "max_output_tokens": stage.get(
                "max_output_tokens",
                model.get("max_output_tokens", 4096),
            ),
            "validation_max_attempts": stage.get(
                "validation_max_attempts",
                config.get("agent", {}).get(
                    "validation_max_attempts",
                    3,
                ),
            ),
            "fallback_to_auto_tool_choice": stage.get(
                "fallback_to_auto_tool_choice",
                True,
            ),
            "reuse_accepted_docstrings": stage.get(
                "reuse_accepted_docstrings",
                True,
            ),
            "strict_tool_schema": True,
            "parallel_tool_calls": False,
        },
        "software": {
            "python_version": platform.python_version(),
            "openai_python_version": (
                importlib.metadata.version("openai")
            ),
        },
        "tool_schema": DOCSTRING_TOOL,
        "functions": {},
    }

    client = create_client(config)
    reuse_enabled = stage.get(
        "reuse_accepted_docstrings",
        True,
    )
    documentation_by_function: dict[
        str,
        dict[str, Any],
    ] = {}

    try:
        for index, function in enumerate(
            functions,
            start=1,
        ):
            function_name = (
                function.analysis.function_name
            )

            reused = None
            if reuse_enabled:
                reused = reusable_documentation(
                    previous_record,
                    source_hash,
                    instructions_hash,
                    function,
                )

            if reused is not None:
                documentation_by_function[
                    function_name
                ] = reused
                record["functions"][function_name] = {
                    "index": index,
                    "status": "reused",
                    "analysis": asdict(
                        function.analysis
                    ),
                    "documentation": reused,
                }
                print(
                    f"[L-A-III] {index}/{len(functions)} "
                    f"reused: {function_name}"
                )
                continue

            documentation, attempts, prompt = (
                generate_documentation(
                    client,
                    config,
                    function,
                )
            )

            documentation_by_function[
                function_name
            ] = documentation
            record["functions"][function_name] = {
                "index": index,
                "status": "accepted",
                "analysis": asdict(
                    function.analysis
                ),
                "prompt": prompt,
                "prompt_sha256": sha256_text(prompt),
                "attempts": attempts,
                "documentation": documentation,
            }

            # Checkpoint accepted work after every function.
            write_json(record_path, record)

            print(
                f"[L-A-III] {index}/{len(functions)} "
                f"accepted: {function_name}"
            )

        codified_source = replace_docstrings(
            original_source,
            functions,
            documentation_by_function,
        )

        validate_codified_source(
            original_source,
            codified_source,
            output_path,
        )

        output_path.write_text(
            codified_source,
            encoding="utf-8",
        )

        record.update(
            {
                "status": "success",
                "completed_at_utc": utc_now(),
                "duration_seconds": round(
                    time.perf_counter() - started,
                    6,
                ),
                "output_sha256": sha256_file(
                    output_path
                ),
                "executable_code_preserved": True,
                "codified_function_count": len(
                    functions
                ),
            }
        )
        write_json(record_path, record)

        print(f"Generated: {output_path}")
        print(f"Generated: {record_path}")
        print(
            f"Codified functions: {len(functions)}"
        )

    except Exception as exc:
        record.update(
            {
                "status": "failed",
                "completed_at_utc": utc_now(),
                "duration_seconds": round(
                    time.perf_counter() - started,
                    6,
                ),
                "failure": {
                    "type": type(exc).__name__,
                    "message": str(exc),
                },
            }
        )
        write_json(record_path, record)
        raise


if __name__ == "__main__":
    main()
