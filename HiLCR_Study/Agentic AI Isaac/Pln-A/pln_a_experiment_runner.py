#!/usr/bin/env python3
"""
Run the Pln-A experiment on three wall-panel examples under two conditions:

1. With_Knowledge:
   Ext-A-II crew mappings and Ext-A-III occupational tasks are supplied.

2. Without_Knowledge:
   Crew mappings and occupational tasks are omitted. The model receives only
   the assembly instance, framing order, and finalized HLAF catalog.

Expected directory structure beside this script:

    pln_a_experiment_runner.py
    Ext_A_II_Output.json
    Ext_A_III_Output.json
        or Ext_A_III_Output(1).json
    KBS_Simplified_Codified.py
    Framing_Order.txt
    Examples/
        No_Open/
            No_Open.json
        1_Door/
            1_Door.json
        1_Door_1_Window/
            1_Door_1_Win.json

Outputs:

    Pln_A_Experiment_Results/
        Experiment_Summary.json
        No_Open/
            With_Knowledge/
                Final_Function_Sequence.py
                Run_Record.json
                Prompt.txt
                Raw_Response.txt
            Without_Knowledge/
                ...
        1_Door/
            ...
        1_Door_1_Window/
            ...

Required environment variable:
    OPENAI_API_KEY

Optional environment variables:
    OPENAI_BASE_URL
    OPENAI_MODEL
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    from openai import OpenAI
except ModuleNotFoundError:
    OpenAI = None


BASE_DIR = Path(__file__).resolve().parent

CREWS_FILE = BASE_DIR / "Ext_A_II_Output.json"
FUNCTIONS_FILE = BASE_DIR / "KBS_Simplified_Codified.py"
FRAMING_ORDER_FILE = BASE_DIR / "Framing_Order.txt"

DEFAULT_OUTPUT_ROOT = BASE_DIR / "Pln_A_Experiment_Results"

EXPERIMENT_CASES: tuple[tuple[str, Path], ...] = (
    (
        "No_Open",
        BASE_DIR / "Examples" / "No_Open" / "No_Open.json",
    ),
    (
        "1_Door",
        BASE_DIR / "Examples" / "1_Door" / "1_Door.json",
    ),
    (
        "1_Door_1_Window",
        BASE_DIR
        / "Examples"
        / "1_Door_1_Window"
        / "1_Door_1_Win.json",
    ),
)

KNOWLEDGE_MODES: tuple[tuple[str, bool], ...] = (
    ("With_Knowledge", True),
    ("Without_Knowledge", False),
)


@dataclass(frozen=True)
class FunctionSpec:
    name: str
    parameter_names: tuple[str, ...]
    parameters_with_none_default: tuple[str, ...]
    signature: str
    docstring: str


@dataclass(frozen=True)
class ModelResult:
    content: str
    response_id: str | None
    model: str | None
    finish_reason: str | None
    usage: dict[str, Any] | None


class AI_Agent:
    def __init__(
        self,
        api_key: str,
        model: str = "o3",
        base_url: str = "https://api.openai.com/v1",
        timeout_seconds: float = 300.0,
    ) -> None:
        if OpenAI is None:
            raise RuntimeError(
                "The openai package is not installed. Run: "
                "pip install -r requirements_pln_a_experiment.txt"
            )

        self.client = OpenAI(
            api_key=api_key,
            base_url=base_url,
            timeout=timeout_seconds,
            max_retries=2,
        )
        self.model = model

    def design_builder(
        self,
        information: str,
    ) -> ModelResult:
        response = self.client.responses.create(
            model=self.model,
            reasoning={
            "effort": "high"
            },
            input=[
            {
                "role": "user",
                "content": information,
            }
            ],
            max_output_tokens=32768,
            store=False,
        )

        choice = response.choices[0]
        content = response.output_text

        if not content:
            raise RuntimeError(
                "The model returned an empty response."
            )

        usage = getattr(response, "usage", None)
        usage_dict = (
            usage.model_dump(mode="json")
            if hasattr(usage, "model_dump")
            else None
        )

        return ModelResult(
            content=content.strip(),
            response_id=getattr(response, "id", None),
            model=getattr(response, "model", None),
            finish_reason=getattr(
                choice,
                "finish_reason",
                None,
            ),
            usage=usage_dict,
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
        json.dump(
            value,
            file,
            indent=2,
            ensure_ascii=False,
        )
        file.write("\n")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(
        value.encode("utf-8")
    ).hexdigest()


def find_tasks_file() -> Path:
    preferred = (
        BASE_DIR / "Ext_A_III_Output.json",
        BASE_DIR / "Ext_A_III_Output(1).json",
    )

    for path in preferred:
        if path.exists():
            return path

    candidates = sorted(
        BASE_DIR.glob("Ext_A_III_Output*.json")
    )

    if not candidates:
        raise FileNotFoundError(
            "No Ext_A_III_Output*.json file was found."
        )

    if len(candidates) > 1:
        names = ", ".join(
            path.name for path in candidates
        )
        raise RuntimeError(
            "Multiple Ext-A-III files were found: "
            f"{names}. Rename the intended file to "
            "Ext_A_III_Output.json."
        )

    return candidates[0]


def validate_required_files(
    tasks_file: Path,
) -> None:
    required = [
        CREWS_FILE,
        tasks_file,
        FUNCTIONS_FILE,
        FRAMING_ORDER_FILE,
        *[
            case_path
            for _, case_path in EXPERIMENT_CASES
        ],
    ]

    missing = [
        str(path.relative_to(BASE_DIR))
        if path.is_relative_to(BASE_DIR)
        else str(path)
        for path in required
        if not path.exists()
    ]

    if missing:
        raise FileNotFoundError(
            "Missing required files:\n- "
            + "\n- ".join(missing)
        )


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
    positional = [
        *node.args.posonlyargs,
        *node.args.args,
    ]

    default_by_name: dict[str, ast.expr | None] = {
        argument.arg: None
        for argument in positional
    }

    if node.args.defaults:
        defaulted_arguments = positional[
            -len(node.args.defaults):
        ]

        for argument, default in zip(
            defaulted_arguments,
            node.args.defaults,
        ):
            default_by_name[argument.arg] = default

    required: list[str] = []

    for argument in positional:
        if argument.arg == "self":
            continue

        default = default_by_name[argument.arg]

        if (
            default is None
            or (
                isinstance(default, ast.Constant)
                and default.value is None
            )
        ):
            required.append(argument.arg)

    for argument, default in zip(
        node.args.kwonlyargs,
        node.args.kw_defaults,
    ):
        if (
            default is None
            or (
                isinstance(default, ast.Constant)
                and default.value is None
            )
        ):
            required.append(argument.arg)

    return tuple(required)


def extract_hlaf_catalog(
    path: Path,
) -> tuple[str, dict[str, FunctionSpec]]:
    source = read_text(path)
    tree = ast.parse(
        source,
        filename=str(path),
    )

    entries: list[str] = []
    specs: dict[str, FunctionSpec] = {}

    for node in tree.body:
        if not isinstance(
            node,
            (ast.FunctionDef, ast.AsyncFunctionDef),
        ):
            continue

        signature = placeholder_signature(node)
        docstring = ast.get_docstring(
            node,
            clean=False,
        )

        if not docstring:
            docstring = (
                "No descriptive docstring is available."
            )

        spec = FunctionSpec(
            name=node.name,
            parameter_names=function_parameter_names(
                node
            ),
            parameters_with_none_default=(
                parameters_with_none_default(node)
            ),
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
        raise ValueError(
            f"No top-level functions were found in "
            f"{path.name}."
        )

    return "\n\n".join(entries), specs


def normalize_crews(data: Any) -> list[Any]:
    if (
        not isinstance(data, dict)
        or not isinstance(data.get("Crews"), list)
    ):
        raise ValueError(
            "Ext_A_II_Output.json must contain a "
            "top-level Crews list."
        )

    return data["Crews"]


def normalize_tasks(data: Any) -> list[Any]:
    if (
        not isinstance(data, dict)
        or not isinstance(data.get("Tasks"), list)
    ):
        raise ValueError(
            "Ext-A-III output must contain a "
            "top-level Tasks list."
        )

    return data["Tasks"]


def strip_code_fences(value: str) -> str:
    stripped = value.strip()

    match = re.fullmatch(
        r"```(?:python)?\s*(.*?)\s*```",
        stripped,
        flags=re.DOTALL | re.IGNORECASE,
    )

    if match:
        return match.group(1).strip()

    return stripped


def knowledge_section(
    include_knowledge: bool,
    crews: list[Any],
    tasks: list[Any],
) -> str:
    if include_knowledge:
        return (
            "Human Trade Knowledge Is Available:\n"
            "Use the following mappings as procedural grounding when "
            "the framing order does not fully specify a component's "
            "internal robotic installation procedure.\n\n"
            "Component-to-Crew Mapping:\n"
            f"{json.dumps(crews, indent=2, ensure_ascii=False)}\n\n"
            "Crew-to-Task Knowledge:\n"
            f"{json.dumps(tasks, indent=2, ensure_ascii=False)}\n"
        )

    return (
        "Human Trade Knowledge Is Not Available:\n"
        "No component-to-crew mapping or occupational task descriptions "
        "are supplied in this experimental condition. Infer the function "
        "sequence only from the Assembly Instance Data, Design Rules, "
        "framing order, and the finalized HLAF descriptions. Do not assume "
        "access to omitted crew or occupational-task data.\n"
    )


def build_prompt(
    *,
    case_name: str,
    assembly_instance: Any,
    design_rules: str,
    hlaf_catalog: str,
    include_knowledge: bool,
    crews: list[Any],
    tasks: list[Any],
    validation_feedback: str = "",
) -> str:
    prompt = (
        "You are Pln-A, a construction robotic planning agent. Generate "
        "a design-driven assembly strategy as an ordered sequence of calls "
        "to the supplied finalized high-level assembly functions for the "
        f"wall-panel case {case_name!r}.\n\n"

        "The Assembly Instance Data contains design-specific element names "
        "or identifiers, element types, center coordinates, dimensions, "
        "and other geometric metadata. Populate function arguments with "
        "exact values from this JSON. Do not invent missing values.\n\n"

        "The Design Rules define installation precedence, left-to-right "
        "directionality, opening logic, grouping, and the overall framing "
        "order. Follow them throughout the sequence.\n\n"

        "The High-Level Assembly Function catalog contains the only "
        "functions that may be called. Use exact function names and exact "
        "keyword argument names. Do not define new functions, import "
        "modules, call low-level robot methods, or include unsupported "
        "helper calls.\n\n"

        "The output must be a directly usable Python sequence. It may "
        "contain comments, direct HLAF calls, and assignments that capture "
        "a returned value for use by a later HLAF call. Do not wrap the "
        "sequence in a function, class, loop, or conditional. Use keyword "
        "arguments rather than positional arguments. Return Python code "
        "only, with no Markdown fences or explanatory prose.\n\n"

        "Assembly Instance Data:\n"
        f"{json.dumps(assembly_instance, indent=2, ensure_ascii=False)}\n\n"

        "Design Rules / Framing Order:\n"
        f"{design_rules.strip()}\n\n"

        "Finalized High-Level Assembly Functions:\n"
        f"{hlaf_catalog}\n\n"

        f"{knowledge_section(include_knowledge, crews, tasks)}"
    )

    if validation_feedback:
        prompt += (
            "\n\nThe previous generated sequence was rejected by the "
            "local validator for the following reason:\n"
            f"{validation_feedback}\n"
            "Return a corrected complete sequence for the same case."
        )

    return prompt


def function_call_name(
    call: ast.Call,
) -> str:
    if not isinstance(call.func, ast.Name):
        raise ValueError(
            "Every call must directly invoke a supplied HLAF "
            "function by name."
        )

    return call.func.id


def validate_call(
    call: ast.Call,
    specs: dict[str, FunctionSpec],
    assigned_names: set[str],
) -> str:
    function_name = function_call_name(call)

    if function_name not in specs:
        raise ValueError(
            f"Unsupported function call: {function_name!r}."
        )

    if call.args:
        raise ValueError(
            f"{function_name} uses positional arguments. "
            "Only keyword arguments are allowed."
        )

    if any(keyword.arg is None for keyword in call.keywords):
        raise ValueError(
            f"{function_name} uses **kwargs expansion, which is "
            "not allowed."
        )

    keyword_names = [
        keyword.arg
        for keyword in call.keywords
        if keyword.arg is not None
    ]

    if len(keyword_names) != len(set(keyword_names)):
        raise ValueError(
            f"{function_name} repeats a keyword argument."
        )

    spec = specs[function_name]
    unknown = sorted(
        set(keyword_names) - set(spec.parameter_names)
    )

    if unknown:
        raise ValueError(
            f"{function_name} uses unknown keyword arguments: "
            f"{unknown}."
        )

    missing_none_parameters = sorted(
        set(spec.parameters_with_none_default)
        - set(keyword_names)
    )

    if missing_none_parameters:
        raise ValueError(
            f"{function_name} omits parameters whose declared "
            f"default is None: {missing_none_parameters}."
        )

    for keyword in call.keywords:
        for child in ast.walk(keyword.value):
            if isinstance(child, ast.Call):
                raise ValueError(
                    f"{function_name} contains a nested function "
                    "call inside an argument."
                )

            if isinstance(child, ast.Name):
                if (
                    child.id not in assigned_names
                    and child.id not in {
                        "True",
                        "False",
                        "None",
                    }
                ):
                    # Constants and literals do not create ast.Name.
                    # A name is therefore expected to be a prior result.
                    raise ValueError(
                        f"{function_name} references variable "
                        f"{child.id!r} before it is assigned by a "
                        "previous HLAF call."
                    )

    return function_name


def validate_generated_sequence(
    generated_code: str,
    specs: dict[str, FunctionSpec],
) -> dict[str, Any]:
    tree = ast.parse(
        generated_code,
        filename="<Pln-A generated sequence>",
    )

    assigned_names: set[str] = set()
    call_names: list[str] = []

    for statement in tree.body:
        call: ast.Call

        if (
            isinstance(statement, ast.Expr)
            and isinstance(statement.value, ast.Call)
        ):
            call = statement.value

        elif (
            isinstance(statement, ast.Assign)
            and isinstance(statement.value, ast.Call)
        ):
            if len(statement.targets) != 1:
                raise ValueError(
                    "A returned HLAF value may be assigned to "
                    "only one variable."
                )

            target = statement.targets[0]
            if not isinstance(target, ast.Name):
                raise ValueError(
                    "HLAF return values must be assigned to a "
                    "simple variable name."
                )

            call = statement.value
            function_name = validate_call(
                call,
                specs,
                assigned_names,
            )
            call_names.append(function_name)
            assigned_names.add(target.id)
            continue

        elif (
            isinstance(statement, ast.AnnAssign)
            and isinstance(statement.value, ast.Call)
        ):
            if not isinstance(statement.target, ast.Name):
                raise ValueError(
                    "Annotated HLAF return values must be assigned "
                    "to a simple variable name."
                )

            call = statement.value
            function_name = validate_call(
                call,
                specs,
                assigned_names,
            )
            call_names.append(function_name)
            assigned_names.add(
                statement.target.id
            )
            continue

        else:
            raise ValueError(
                "The generated file may contain only direct HLAF "
                "calls, assignments receiving HLAF return values, "
                "and comments."
            )

        function_name = validate_call(
            call,
            specs,
            assigned_names,
        )
        call_names.append(function_name)

    if not call_names:
        raise ValueError(
            "The generated sequence contains no HLAF calls."
        )

    return {
        "call_count": len(call_names),
        "function_calls": call_names,
        "unique_function_calls": sorted(
            set(call_names)
        ),
        "assigned_result_variables": sorted(
            assigned_names
        ),
    }


def execute_single_run(
    *,
    agent: AI_Agent,
    case_name: str,
    assembly_path: Path,
    include_knowledge: bool,
    mode_name: str,
    output_root: Path,
    design_rules: str,
    hlaf_catalog: str,
    function_specs: dict[str, FunctionSpec],
    crews: list[Any],
    tasks: list[Any],
    tasks_file: Path,
    max_attempts: int,
    save_prompt: bool,
) -> dict[str, Any]:
    run_started = time.perf_counter()
    started_at = utc_now()

    run_dir = output_root / case_name / mode_name
    run_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    sequence_path = (
        run_dir / "Final_Function_Sequence.py"
    )
    record_path = run_dir / "Run_Record.json"
    prompt_path = run_dir / "Prompt.txt"
    raw_response_path = (
        run_dir / "Raw_Response.txt"
    )

    assembly_instance = read_json(
        assembly_path
    )

    input_files: dict[str, str] = {
        assembly_path.name: sha256_file(
            assembly_path
        ),
        FUNCTIONS_FILE.name: sha256_file(
            FUNCTIONS_FILE
        ),
        FRAMING_ORDER_FILE.name: sha256_file(
            FRAMING_ORDER_FILE
        ),
    }

    if include_knowledge:
        input_files.update(
            {
                CREWS_FILE.name: sha256_file(
                    CREWS_FILE
                ),
                tasks_file.name: sha256_file(
                    tasks_file
                ),
            }
        )

    run_record: dict[str, Any] = {
        "agent": "Pln-A",
        "case": case_name,
        "knowledge_mode": mode_name,
        "knowledge_included": include_knowledge,
        "status": "running",
        "started_at_utc": started_at,
        "completed_at_utc": None,
        "duration_seconds": None,
        "model": agent.model,
        "input_files_sha256": input_files,
        "output_files": {
            "function_sequence": sequence_path.name,
            "run_record": record_path.name,
            "prompt": (
                prompt_path.name
                if save_prompt
                else None
            ),
            "raw_response": (
                raw_response_path.name
                if save_prompt
                else None
            ),
        },
        "attempts": [],
    }
    write_json(record_path, run_record)

    feedback = ""
    last_error = "Unknown generation error."

    for attempt_number in range(
        1,
        max_attempts + 1,
    ):
        attempt_started = time.perf_counter()
        prompt = build_prompt(
            case_name=case_name,
            assembly_instance=assembly_instance,
            design_rules=design_rules,
            hlaf_catalog=hlaf_catalog,
            include_knowledge=include_knowledge,
            crews=crews,
            tasks=tasks,
            validation_feedback=feedback,
        )

        try:
            result = agent.design_builder(prompt)
            generated_code = strip_code_fences(
                result.content
            )

            validation = validate_generated_sequence(
                generated_code,
                function_specs,
            )

            generated_code = (
                f"# Pln-A case: {case_name}\n"
                f"# Knowledge mode: {mode_name}\n\n"
                + generated_code.rstrip()
                + "\n"
            )

            # Validate once more after adding comments.
            compile(
                generated_code,
                str(sequence_path),
                "exec",
            )

            sequence_path.write_text(
                generated_code,
                encoding="utf-8",
            )

            if save_prompt:
                prompt_path.write_text(
                    prompt,
                    encoding="utf-8",
                )
                raw_response_path.write_text(
                    result.content + "\n",
                    encoding="utf-8",
                )

            attempt_record = {
                "attempt": attempt_number,
                "status": "accepted",
                "response_id": result.response_id,
                "returned_model": result.model,
                "finish_reason": (
                    result.finish_reason
                ),
                "usage": result.usage,
                "duration_seconds": round(
                    time.perf_counter()
                    - attempt_started,
                    6,
                ),
                "prompt_sha256": sha256_text(
                    prompt
                ),
                "raw_response_sha256": (
                    sha256_text(result.content)
                ),
                "validation": validation,
            }
            run_record["attempts"].append(
                attempt_record
            )

            run_record.update(
                {
                    "status": "success",
                    "completed_at_utc": utc_now(),
                    "duration_seconds": round(
                        time.perf_counter()
                        - run_started,
                        6,
                    ),
                    "final_sequence_sha256": (
                        sha256_file(sequence_path)
                    ),
                    "final_validation": validation,
                }
            )
            write_json(record_path, run_record)

            print(
                f"[Pln-A] {case_name} / {mode_name}: "
                f"success ({validation['call_count']} calls)"
            )

            return run_record

        except Exception as exc:
            last_error = (
                f"{type(exc).__name__}: {exc}"
            )
            run_record["attempts"].append(
                {
                    "attempt": attempt_number,
                    "status": "rejected",
                    "duration_seconds": round(
                        time.perf_counter()
                        - attempt_started,
                        6,
                    ),
                    "error": last_error,
                    "prompt_sha256": sha256_text(
                        prompt
                    ),
                }
            )
            write_json(record_path, run_record)

            print(
                f"[Pln-A] {case_name} / {mode_name}, "
                f"attempt {attempt_number}/{max_attempts} "
                f"rejected: {last_error}",
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
                    f"Generation failed after "
                    f"{max_attempts} attempts. "
                    f"Last error: {last_error}"
                ),
            },
        }
    )
    write_json(record_path, run_record)

    print(
        f"[Pln-A] {case_name} / {mode_name}: failed",
        file=sys.stderr,
    )

    return run_record


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run six Pln-A experiments: three assembly "
            "examples, each with and without human trade "
            "knowledge."
        )
    )
    parser.add_argument(
        "--model",
        default=os.getenv(
            "OPENAI_MODEL",
            "o3",
        ),
        help="Model identifier. Default: o3.",
    )
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help=(
            "Experiment output directory. Default: "
            "Pln_A_Experiment_Results beside this script."
        ),
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=3,
        help=(
            "Maximum generation/validation attempts for "
            "each of the six runs. Default: 3."
        ),
    )
    parser.add_argument(
        "--no-prompt-files",
        action="store_true",
        help=(
            "Do not save Prompt.txt and Raw_Response.txt "
            "for each run."
        ),
    )
    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help=(
            "Stop immediately when one experimental run "
            "fails. By default, remaining runs continue."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    experiment_started = time.perf_counter()

    if args.max_attempts < 1:
        raise ValueError(
            "--max-attempts must be at least 1."
        )

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError(
            "OPENAI_API_KEY is not set."
        )

    base_url = os.getenv(
        "OPENAI_BASE_URL",
        "https://api.openai.com/v1",
    )

    tasks_file = find_tasks_file()
    validate_required_files(tasks_file)

    crews = normalize_crews(
        read_json(CREWS_FILE)
    )
    tasks = normalize_tasks(
        read_json(tasks_file)
    )
    design_rules = read_text(
        FRAMING_ORDER_FILE
    )
    hlaf_catalog, function_specs = (
        extract_hlaf_catalog(FUNCTIONS_FILE)
    )

    output_root = Path(
        args.output_root
    ).expanduser()
    if not output_root.is_absolute():
        output_root = BASE_DIR / output_root
    output_root = output_root.resolve()
    output_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    agent = AI_Agent(
        api_key=api_key,
        model=args.model,
        base_url=base_url,
    )

    summary: dict[str, Any] = {
        "agent": "Pln-A",
        "experiment": (
            "Three wall-panel cases with and without "
            "human trade knowledge"
        ),
        "status": "running",
        "started_at_utc": utc_now(),
        "completed_at_utc": None,
        "duration_seconds": None,
        "model": args.model,
        "base_url": base_url,
        "cases": {},
        "successful_runs": 0,
        "failed_runs": 0,
        "total_runs": (
            len(EXPERIMENT_CASES)
            * len(KNOWLEDGE_MODES)
        ),
        "shared_input_files_sha256": {
            FUNCTIONS_FILE.name: sha256_file(
                FUNCTIONS_FILE
            ),
            FRAMING_ORDER_FILE.name: sha256_file(
                FRAMING_ORDER_FILE
            ),
            CREWS_FILE.name: sha256_file(
                CREWS_FILE
            ),
            tasks_file.name: sha256_file(
                tasks_file
            ),
        },
        "hlaf_function_count": len(
            function_specs
        ),
    }

    summary_path = (
        output_root / "Experiment_Summary.json"
    )
    write_json(summary_path, summary)

    any_failure = False

    for case_name, assembly_path in EXPERIMENT_CASES:
        summary["cases"][case_name] = {}

        for mode_name, include_knowledge in (
            KNOWLEDGE_MODES
        ):
            record = execute_single_run(
                agent=agent,
                case_name=case_name,
                assembly_path=assembly_path,
                include_knowledge=include_knowledge,
                mode_name=mode_name,
                output_root=output_root,
                design_rules=design_rules,
                hlaf_catalog=hlaf_catalog,
                function_specs=function_specs,
                crews=crews,
                tasks=tasks,
                tasks_file=tasks_file,
                max_attempts=args.max_attempts,
                save_prompt=(
                    not args.no_prompt_files
                ),
            )

            summary["cases"][case_name][
                mode_name
            ] = {
                "status": record["status"],
                "duration_seconds": record[
                    "duration_seconds"
                ],
                "output_directory": str(
                    (
                        output_root
                        / case_name
                        / mode_name
                    ).relative_to(output_root)
                ),
                "call_count": record.get(
                    "final_validation",
                    {},
                ).get("call_count"),
            }

            if record["status"] == "success":
                summary["successful_runs"] += 1
            else:
                summary["failed_runs"] += 1
                any_failure = True

            write_json(summary_path, summary)

            if (
                any_failure
                and args.stop_on_error
            ):
                break

        if any_failure and args.stop_on_error:
            break

    summary.update(
        {
            "status": (
                "success"
                if not any_failure
                else "completed_with_failures"
            ),
            "completed_at_utc": utc_now(),
            "duration_seconds": round(
                time.perf_counter()
                - experiment_started,
                6,
            ),
        }
    )
    write_json(summary_path, summary)

    print(f"Generated: {summary_path}")
    print(
        f"Successful runs: "
        f"{summary['successful_runs']}/"
        f"{summary['total_runs']}"
    )

    if any_failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
