#!/usr/bin/env python3
"""
Pln-A: generate a design-driven robotic assembly sequence.

All input files are read from the same directory as this script.

Required files:
    Ext_A_II_Output.json
    Ext_A_III_Output.json
        or Ext_A_III_Output(1).json
    KBS_Simplified_Codified.py
    Framing_Order.txt
    <assembly-instance JSON file>

The assembly-instance file can be provided with:
    python3 pln_a_agent.py --assembly-instance 1_Door_1_Win.json

Outputs:
    Pln_A_Generated_Sequence.py
    Pln_A_Run_Record.json
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

OUTPUT_FILE = BASE_DIR / "Pln_A_Generated_Sequence.py"
RUN_RECORD_FILE = BASE_DIR / "Pln_A_Run_Record.json"


class AI_Agent:
    def __init__(
        self,
        api_key: str,
        model: str = "o3",
        base_url: str = "https://api.openai.com/v1",
    ) -> None:
        if OpenAI is None:
            raise RuntimeError(
                "The openai package is not installed. Run: "
                "pip install -r requirements_pln_a.txt"
            )

        self.api_key = api_key
        self.client = OpenAI(
            api_key=api_key,
            base_url=base_url,
        )
        self.model = model

    def change_agent_model(
        self,
        model: str | None = None,
        base_url: str | None = None,
    ) -> None:
        if model is None or base_url is None:
            raise ValueError(
                "Both model and base_url are required."
            )

        self.client = OpenAI(
            api_key=self.api_key,
            base_url=base_url,
        )
        self.model = model

    def design_builder(
        self,
        information: str,
        model: str | None = None,
    ) -> str:
        if model is not None:
            self.model = model

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {
                    "role": "user",
                    "content": information,
                }
            ],
        )

        message = response.choices[0].message.content
        if not message:
            raise RuntimeError(
                "The model returned an empty response."
            )

        return message.strip()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def find_tasks_file() -> Path:
    candidates = [
        BASE_DIR / "Ext_A_III_Output.json",
        BASE_DIR / "Ext_A_III_Output(1).json",
    ]

    for candidate in candidates:
        if candidate.exists():
            return candidate

    matches = sorted(
        BASE_DIR.glob("Ext_A_III_Output*.json")
    )

    if not matches:
        raise FileNotFoundError(
            "No Ext_A_III_Output*.json file was found."
        )

    if len(matches) > 1:
        names = ", ".join(
            path.name for path in matches
        )
        raise RuntimeError(
            "Multiple Ext-A-III files were found: "
            f"{names}. Keep only the intended file or rename "
            "it to Ext_A_III_Output.json."
        )

    return matches[0]


def discover_assembly_instance(
    requested_name: str | None,
) -> Path:
    if requested_name:
        path = Path(requested_name).expanduser()
        if not path.is_absolute():
            path = BASE_DIR / path

        if not path.exists():
            raise FileNotFoundError(
                f"Assembly-instance file not found: {path}"
            )

        return path.resolve()

    preferred_names = [
        "Assembly_Instance_Data.json",
        "AssemblyInstanceData.json",
        "Design_Instance.json",
        "Design.json",
        "1_Door_1_Win.json",
        "1_Door_1_Window.json",
    ]

    for name in preferred_names:
        candidate = BASE_DIR / name
        if candidate.exists():
            return candidate.resolve()

    excluded_names = {
        "Ext_A_II_Output.json",
        "Ext_A_III_Output.json",
        "Ext_A_III_Output(1).json",
        "config.json",
        "Material.json",
        "L_A_I_Output.json",
        "L_A_II_Output.json",
        "AI_HLAF_Generation_Run_Record.json",
        "L_A_III_KBS_Run_Record.json",
        "Pln_A_Run_Record.json",
    }

    candidates = [
        path.resolve()
        for path in BASE_DIR.glob("*.json")
        if path.name not in excluded_names
        and "Run_Record" not in path.name
        and not path.name.startswith(
            (
                "Primitive_",
                "Agent_Description",
                "Ext_A_I",
            )
        )
    ]

    if len(candidates) == 1:
        return candidates[0]

    if not candidates:
        raise FileNotFoundError(
            "No assembly-instance JSON file was found. "
            "Place it beside this script and run with "
            "--assembly-instance <filename>."
        )

    names = ", ".join(
        path.name for path in candidates
    )
    raise RuntimeError(
        "The assembly-instance file is ambiguous. "
        f"Candidate JSON files: {names}. Run with "
        "--assembly-instance <filename>."
    )


def validate_required_files(
    tasks_file: Path,
    assembly_file: Path,
) -> None:
    required = [
        CREWS_FILE,
        tasks_file,
        FUNCTIONS_FILE,
        FRAMING_ORDER_FILE,
        assembly_file,
    ]

    missing = [
        path.name
        for path in required
        if not path.exists()
    ]

    if missing:
        raise FileNotFoundError(
            "Missing required files: "
            + ", ".join(missing)
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

    lines = rendered.splitlines()
    signature_lines: list[str] = []

    for line in lines:
        if line.strip() == "pass":
            break
        signature_lines.append(line)

    return "\n".join(signature_lines).strip()


def extract_hlaf_catalog(path: Path) -> str:
    """
    Extract only function signatures and docstrings.

    The Pln-A table defines the HLAF input as function names, input
    arguments, and descriptive information. The executable function bodies
    are intentionally excluded to reduce prompt size.
    """
    source = read_text(path)
    tree = ast.parse(
        source,
        filename=str(path),
    )

    entries: list[str] = []

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
            f"No top-level functions were found in {path.name}."
        )

    return "\n\n".join(entries)


def normalize_crews(data: Any) -> list[Any]:
    if not isinstance(data, dict):
        raise TypeError(
            "Ext-A-II output must be a JSON object."
        )

    crews = data.get("Crews")
    if not isinstance(crews, list):
        raise ValueError(
            "Ext-A-II output does not contain a Crews list."
        )

    return crews


def normalize_tasks(data: Any) -> list[Any]:
    if not isinstance(data, dict):
        raise TypeError(
            "Ext-A-III output must be a JSON object."
        )

    tasks = data.get("Tasks")
    if not isinstance(tasks, list):
        raise ValueError(
            "Ext-A-III output does not contain a Tasks list."
        )

    return tasks


def strip_code_fences(value: str) -> str:
    value = value.strip()

    match = re.fullmatch(
        r"```(?:python)?\s*(.*?)\s*```",
        value,
        flags=re.DOTALL | re.IGNORECASE,
    )

    if match:
        return match.group(1).strip()

    return value


def build_prompt(
    assembly_instance: Any,
    design_rules: str,
    hlaf_catalog: str,
    tasks: list[Any],
    crews: list[Any],
) -> str:
    return (
        "Assume that you are a construction robotic engineer who must "
        "generate a design-driven assembly strategy as an ordered sequence "
        "of function calls for the supplied wood-frame wall-panel design.\n\n"

        "The available high-level assembly functions are general robotic "
        "station capabilities. Each function is provided through its exact "
        "name, Python signature, required arguments, and descriptive "
        "docstring. Use only these supplied functions in the generated "
        "sequence.\n\n"

        "The Assembly Instance Data contains the design-specific element "
        "names or identifiers, element types, center coordinates, "
        "dimensions, and any other geometric metadata. Use exact values "
        "from this JSON to populate every required function argument. Do "
        "not invent missing coordinates, dimensions, identifiers, or "
        "function parameters.\n\n"

        "Follow the supplied Design Rules and framing order. The rules "
        "define precedence, directionality, opening logic, and grouping. "
        "When the rules do not explicitly provide a component's internal "
        "installation procedure, use the supplied human crew mappings and "
        "occupational tasks as procedural grounding for selecting and "
        "ordering the available robotic functions.\n\n"

        "Generate a ready-to-run Python script composed exclusively of calls "
        "to the supplied high-level assembly functions, arranged in a "
        "logically valid assembly order. Do not redefine the functions. Do "
        "not add unsupported low-level robot commands. Use exact Python "
        "function names and keyword argument names. Include brief comments "
        "that identify the element being assembled. Return Python code only, "
        "without Markdown fences or explanatory prose.\n\n"

        "Assembly Instance Data:\n"
        f"{json.dumps(assembly_instance, indent=2, ensure_ascii=False)}\n\n"

        "Design Rules / Framing Order:\n"
        f"{design_rules.strip()}\n\n"

        "High-Level Assembly Functions:\n"
        f"{hlaf_catalog}\n\n"

        "Component-to-Crew Mapping:\n"
        f"{json.dumps(crews, indent=2, ensure_ascii=False)}\n\n"

        "Crew-to-Task Knowledge:\n"
        f"{json.dumps(tasks, indent=2, ensure_ascii=False)}\n"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a design-driven sequence of finalized HLAF "
            "function calls."
        )
    )
    parser.add_argument(
        "--assembly-instance",
        help=(
            "Assembly-instance JSON filename or path. The file is "
            "otherwise auto-discovered in the script directory."
        ),
    )
    parser.add_argument(
        "--model",
        default=os.getenv(
            "OPENAI_MODEL",
            "o3",
        ),
        help="OpenAI model identifier. Default: o3.",
    )
    parser.add_argument(
        "--print-prompt",
        action="store_true",
        help="Print the complete generated prompt.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    started = time.perf_counter()

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print(
            "OPENAI_API_KEY was not found in the environment.",
            file=sys.stderr,
        )
        sys.exit(1)

    base_url = os.getenv(
        "OPENAI_BASE_URL",
        "https://api.openai.com/v1",
    )

    tasks_file = find_tasks_file()
    assembly_file = discover_assembly_instance(
        args.assembly_instance
    )

    validate_required_files(
        tasks_file,
        assembly_file,
    )

    crews_data = read_json(CREWS_FILE)
    tasks_data = read_json(tasks_file)
    assembly_instance = read_json(assembly_file)

    crews = normalize_crews(crews_data)
    tasks = normalize_tasks(tasks_data)

    design_rules = read_text(
        FRAMING_ORDER_FILE
    )
    hlaf_catalog = extract_hlaf_catalog(
        FUNCTIONS_FILE
    )

    prompt = build_prompt(
        assembly_instance=assembly_instance,
        design_rules=design_rules,
        hlaf_catalog=hlaf_catalog,
        tasks=tasks,
        crews=crews,
    )

    if args.print_prompt:
        print(prompt)

    agent = AI_Agent(
        api_key=api_key,
        model=args.model,
        base_url=base_url,
    )

    generated = agent.design_builder(
        information=prompt,
    )
    generated_code = strip_code_fences(
        generated
    )

    # Ensure that the returned sequence is syntactically valid Python.
    compile(
        generated_code,
        str(OUTPUT_FILE),
        "exec",
    )

    OUTPUT_FILE.write_text(
        generated_code + "\n",
        encoding="utf-8",
    )

    run_record = {
        "agent": "Pln-A",
        "status": "success",
        "started_at_utc": utc_now(),
        "completed_at_utc": utc_now(),
        "duration_seconds": round(
            time.perf_counter() - started,
            6,
        ),
        "model": args.model,
        "base_url": base_url,
        "input_files": {
            CREWS_FILE.name: sha256_file(
                CREWS_FILE
            ),
            tasks_file.name: sha256_file(
                tasks_file
            ),
            FUNCTIONS_FILE.name: sha256_file(
                FUNCTIONS_FILE
            ),
            FRAMING_ORDER_FILE.name: sha256_file(
                FRAMING_ORDER_FILE
            ),
            assembly_file.name: sha256_file(
                assembly_file
            ),
        },
        "input_counts": {
            "crew_mappings": len(crews),
            "crew_task_groups": len(tasks),
            "hlaf_functions": hlaf_catalog.count(
                "Function Name:"
            ),
        },
        "output_file": OUTPUT_FILE.name,
        "output_sha256": sha256_file(
            OUTPUT_FILE
        ),
    }

    with RUN_RECORD_FILE.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            run_record,
            file,
            indent=2,
            ensure_ascii=False,
        )
        file.write("\n")

    print(f"Generated: {OUTPUT_FILE}")
    print(f"Generated: {RUN_RECORD_FILE}")
    print(
        "Time it took to generate: "
        f"{run_record['duration_seconds']} seconds"
    )


if __name__ == "__main__":
    main()
