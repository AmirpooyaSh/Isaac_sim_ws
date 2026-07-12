#!/usr/bin/env python3
"""
AI-based Sb-A, Pb-A, and Co-A HLAF generator.

Unlike the deterministic generator, this program sends the assigned HLAF
procedures to three OpenAI o3 agents:

    1. Sb-A generates all validated Pick code blocks.
    2. Pb-A generates all Place and Nail code blocks.
    3. Co-A generates all synchronized collaborative Pass code blocks.

The locally executed orchestrator validates the returned Python, combines the
agent-generated blocks in the original L-A-II order, and writes one HLAF file.

Place these files beside this script:
    L_A_II_Output.json
    Primitive_Library_Definitions.json
    config.json

Required environment variable:
    OPENAI_API_KEY

Optional environment variable:
    OPENAI_BASE_URL

Created files:
    AI_Generated_HLAFs.py
    AI_HLAF_Generation_Run_Record.json
"""

from __future__ import annotations

import ast
import hashlib
import importlib.metadata
import json
import keyword
import os
import platform
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from openai import (
    AuthenticationError,
    BadRequestError,
    OpenAI,
)


ROOT = Path(__file__).resolve().parent

FLOW_FILE = ROOT / "L_A_II_Output.json"
DEFINITIONS_FILE = ROOT / "Primitive_Library_Definitions.json"
CONFIG_FILE = ROOT / "config.json"

OUTPUT_FILE = ROOT / "AI_Generated_HLAFs.py"
RUN_RECORD_FILE = ROOT / "AI_HLAF_Generation_Run_Record.json"

CANONICAL_VARIABLE_ORDER = ("L", "W", "H", "X", "Y", "Z")

DEFAULT_RUNTIME_NAMES = {
    "Robot_1": "IRB6620_R1",
    "Robot_2": "IRB6620_R2",
}

BLOCK_TOOL_NAME = "submit_hlaf_code_blocks"

BLOCK_TOOL: dict[str, Any] = {
    "type": "function",
    "name": BLOCK_TOOL_NAME,
    "description": (
        "Submit the generated Python code lines for every assigned HLAF block."
    ),
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "Blocks": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "block_id": {
                            "type": "string",
                            "description": (
                                "The exact block_id supplied in the task."
                            ),
                        },
                        "code_lines": {
                            "type": "array",
                            "items": {"type": "string"},
                            "description": (
                                "Python statements only, one source line per "
                                "array entry, without Markdown fences."
                            ),
                        },
                    },
                    "required": ["block_id", "code_lines"],
                    "additionalProperties": False,
                },
            }
        },
        "required": ["Blocks"],
        "additionalProperties": False,
    },
}


COMMON_LOW_LEVEL_API = r"""
The robot controllers expose only these relevant low-level functions:

plan(
    tcp_name: str,
    target_pose: np.array,
    target_orientation: np.array
)

render_exec(
    renderInstance: bool = True,
    Show_Sphere: Optional[bool] = True,
    is_synchronizer: bool = False
)

move_to_home(
    if_show_spheres: bool = False,
    Customized_JS: List[float] = [0, -0.5, 0.5, 0, 0, 0],
    removing_primitives=[]
)

eef_attach(
    r_name: str,
    tool_name: str,
    attaching_object_name: str
)

eef_detach(
    r_name: str,
    tool_name: str,
    detaching_object_name: str
)

Rules shared by all agents:
- Generate Python statements only; do not generate imports, functions, classes,
  Markdown, prose, placeholders, or ellipses.
- Use only the controller variable, tool id, runtime robot name, pose
  expressions, quaternion, and element_object_name supplied in each task.
- Every low-level call must be invoked as a method of the supplied controller
  variable, such as robot_2.plan(...) or robot_2.eef_detach(...). Never emit
  plan(...), render_exec(...), move_to_home(...), eef_attach(...), or
  eef_detach(...) as bare functions.
- Quaternion arrays must use [w, x, y, z] order.
- Call render_exec immediately after each plan.
- Pass only tcp_name, target_pose, and target_orientation to plan.
- Use np.array([...], dtype=float) for both pose and orientation.
- Do not invent or change pose expressions.
- move_to_home performs its own execution; never follow it with render_exec.
""".strip()


SB_A_INSTRUCTIONS = f"""
You are Sb-A-II, the Simulation-Based HLAF Generation Agent.

The recursive pose-correction stage Sb-A-I has already completed. Every Pick
pose supplied to you is the final validated TCP pose.

For each assigned Pick block, generate exactly this behavior:
1. Call <robot>.plan with the supplied tool id, position, and quaternion.
2. Call <robot>.render_exec().
3. Call <robot>.eef_attach with the supplied runtime robot name, tool id, and
   element_object_name.
4. Call <robot>.move_to_home().

{COMMON_LOW_LEVEL_API}

Return every requested block through {BLOCK_TOOL_NAME}.
""".strip()


PB_A_INSTRUCTIONS = f"""
You are Pb-A, the Parameter-Based HLAF Generation Agent.

You receive Place and Nail procedures.

For each Place block:
1. plan to the supplied pose;
2. render_exec;
3. eef_detach the element using the supplied runtime robot name and tool id;
4. move_to_home.

For each Nail block:
1. plan to the supplied nail pose;
2. render_exec;
3. move_to_home.

Do not call eef_attach or eef_detach for Nail because those functions represent
grasp attachment and placement release, not nail-gun activation.

{COMMON_LOW_LEVEL_API}

Return every requested block through {BLOCK_TOOL_NAME}.
""".strip()


CO_A_INSTRUCTIONS = f"""
You are Co-A, the Collaborative HLAF Generation Agent.

Each assigned task contains a giver and receiver at the same Pass pose.

For every collaborative block, follow this exact structural pattern:

<giver_controller>.plan(...)
<receiver_controller>.plan(...)

with ThreadPoolExecutor(max_workers=2) as executor:
    giver_exec = executor.submit(
        <giver_controller>.render_exec,
        is_synchronizer=False,
    )
    receiver_exec = executor.submit(
        <receiver_controller>.render_exec,
        is_synchronizer=True,
    )
    giver_exec.result()
    receiver_exec.result()

<receiver_controller>.eef_attach(...)
<giver_controller>.eef_detach(...)

with ThreadPoolExecutor(max_workers=2) as executor:
    giver_home = executor.submit(
        <giver_controller>.move_to_home
    )
    receiver_home = executor.submit(
        <receiver_controller>.move_to_home
    )
    giver_home.result()
    receiver_home.result()

Additional Co-A rules:
- Use ThreadPoolExecutor exactly as shown; do not prefix it with a module name.
- Do not call executor.shutdown(). The context manager performs shutdown.
- Do not replace ThreadPoolExecutor with another concurrency mechanism.
- Attach the receiver before detaching the giver.

Use the local temporary names:
    giver_exec
    receiver_exec
    giver_home
    receiver_home
    executor

{COMMON_LOW_LEVEL_API}

Return every requested block through {BLOCK_TOOL_NAME}.
""".strip()


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


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def usage_to_dict(response: Any) -> dict[str, Any] | None:
    usage = getattr(response, "usage", None)
    if hasattr(usage, "model_dump"):
        return usage.model_dump(mode="json")
    return None


def safe_identifier(value: str, max_length: int = 76) -> str:
    identifier = re.sub(
        r"[^A-Za-z0-9]+",
        "_",
        value,
    ).strip("_").lower()
    identifier = re.sub(r"_+", "_", identifier)

    if not identifier:
        identifier = "hlaf"
    if identifier[0].isdigit():
        identifier = f"hlaf_{identifier}"
    if keyword.iskeyword(identifier):
        identifier = f"{identifier}_hlaf"

    return identifier[:max_length].rstrip("_")


def indent(lines: Iterable[str], spaces: int = 4) -> list[str]:
    prefix = " " * spaces
    return [
        prefix + line if line else ""
        for line in lines
    ]


def normalize_agent(agent_name: str) -> str:
    lowered = agent_name.casefold()

    if "simulation-based" in lowered or "sb-a" in lowered:
        return "Sb-A"
    if "parameter-based" in lowered or "pb-a" in lowered:
        return "Pb-A"
    if "collaborative" in lowered or "co-a" in lowered:
        return "Co-A"

    raise ValueError(f"Unknown agent name: {agent_name}")


def pose_action(pose_name: str) -> str:
    lowered = pose_name.casefold()

    if "_pick_pose" in lowered:
        return "Pick"
    if "_place_pose" in lowered:
        return "Place"
    if "_pass_pose" in lowered:
        return "Pass"
    if "_nail_pose" in lowered:
        return "Nail"

    raise ValueError(
        f"Cannot infer an action from pose name {pose_name!r}."
    )


def controller_variable(robot: str) -> str:
    return safe_identifier(robot)


def reverse_tool_map(
    definitions: dict[str, dict[str, str]],
) -> dict[str, dict[str, str]]:
    result: dict[str, dict[str, str]] = {}

    for robot, tools in definitions.items():
        result[robot] = {
            tool_name: tool_id
            for tool_id, tool_name in tools.items()
        }

    return result


def resolve_tool_id(
    reverse_tools: dict[str, dict[str, str]],
    robot: str,
    tool_name: str,
) -> str:
    try:
        return reverse_tools[robot][tool_name]
    except KeyError as exc:
        raise KeyError(
            f"No tool id found for {robot} / {tool_name}."
        ) from exc


def get_model_profile(config: dict[str, Any]) -> dict[str, Any]:
    profile_name = config["active_model_profile"]
    return config["model_profiles"][profile_name]


def create_client(config: dict[str, Any]) -> OpenAI:
    api = config.get("api", {})

    options: dict[str, Any] = {
        "api_key": os.environ["OPENAI_API_KEY"],
        "timeout": api.get("timeout_seconds", 180),
        "max_retries": api.get("sdk_max_retries", 2),
    }

    base_url = os.getenv("OPENAI_BASE_URL") or api.get("base_url")
    if base_url:
        options["base_url"] = base_url

    return OpenAI(**options)


def request_parameters(
    config: dict[str, Any],
    instructions: str,
    prompt: str,
    force_tool: bool,
) -> dict[str, Any]:
    model = get_model_profile(config)
    generation = config.get("hlaf_generation", {})

    request: dict[str, Any] = {
        "model": model["identifier"],
        "instructions": instructions,
        "input": prompt,
        "tools": [BLOCK_TOOL],
        "tool_choice": (
            {
                "type": "function",
                "name": BLOCK_TOOL_NAME,
            }
            if force_tool
            else "auto"
        ),
        "parallel_tool_calls": False,
        "max_output_tokens": generation.get(
            "agent_max_output_tokens",
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


def extract_blocks(response: Any) -> dict[str, list[str]]:
    for item in response.output:
        if (
            item.type == "function_call"
            and item.name == BLOCK_TOOL_NAME
        ):
            arguments = json.loads(item.arguments)
            blocks: dict[str, list[str]] = {}

            for block in arguments["Blocks"]:
                block_id = block["block_id"]
                code_lines = block["code_lines"]

                if block_id in blocks:
                    raise ValueError(
                        f"Duplicate returned block_id: {block_id}"
                    )

                blocks[block_id] = code_lines

            return blocks

    raise RuntimeError(
        f"The model did not call {BLOCK_TOOL_NAME}."
    )


@dataclass(frozen=True)
class Procedure:
    robot: str
    tool_name: str
    element: str
    pose: dict[str, Any]
    agent_name: str

    @classmethod
    def from_json(cls, value: list[Any]) -> "Procedure":
        if len(value) != 5:
            raise ValueError(
                "Every procedure must have five fields."
            )

        robot, tool_name, element, pose, agent_name = value

        return cls(
            robot=robot,
            tool_name=tool_name,
            element=element,
            pose=pose,
            agent_name=agent_name,
        )

    @property
    def agent(self) -> str:
        return normalize_agent(self.agent_name)

    @property
    def action(self) -> str:
        return pose_action(self.pose["name"])


@dataclass(frozen=True)
class AgentTask:
    block_id: str
    agent: str
    action: str
    flow_index: int
    procedure_indexes: tuple[int, ...]
    element: str
    sub_step: str
    payload: dict[str, Any]


def task_payload_for_procedure(
    procedure: Procedure,
    reverse_tools: dict[str, dict[str, str]],
    runtime_names: dict[str, str],
) -> dict[str, Any]:
    return {
        "robot": procedure.robot,
        "controller_variable": controller_variable(
            procedure.robot
        ),
        "runtime_robot_name": runtime_names[
            procedure.robot
        ],
        "tool_name": procedure.tool_name,
        "tool_id": resolve_tool_id(
            reverse_tools,
            procedure.robot,
            procedure.tool_name,
        ),
        "element": procedure.element,
        "pose": procedure.pose,
    }


def build_tasks(
    flows: list[dict[str, Any]],
    reverse_tools: dict[str, dict[str, str]],
    runtime_names: dict[str, str],
) -> tuple[
    dict[str, list[AgentTask]],
    dict[tuple[int, int], str],
]:
    tasks = {
        "Sb-A": [],
        "Pb-A": [],
        "Co-A": [],
    }
    procedure_to_block: dict[tuple[int, int], str] = {}

    for flow_index, flow_item in enumerate(flows, start=1):
        procedures = [
            Procedure.from_json(item)
            for item in flow_item["HLAF Flow"]
        ]

        procedure_index = 0

        while procedure_index < len(procedures):
            procedure = procedures[procedure_index]
            human_index = procedure_index + 1

            if procedure.agent == "Co-A":
                if procedure_index + 1 >= len(procedures):
                    raise ValueError(
                        "Collaborative procedure has no paired procedure."
                    )

                receiver = procedures[procedure_index + 1]

                if receiver.agent != "Co-A":
                    raise ValueError(
                        "Collaborative procedure pair is not consecutive."
                    )
                if procedure.pose["name"] != receiver.pose["name"]:
                    raise ValueError(
                        "Collaborative pair uses different pass poses."
                    )

                block_id = (
                    f"flow_{flow_index:03d}_"
                    f"procedures_{human_index:03d}_"
                    f"{human_index + 1:03d}_co_a"
                )

                payload = {
                    "giver": task_payload_for_procedure(
                        procedure,
                        reverse_tools,
                        runtime_names,
                    ),
                    "receiver": task_payload_for_procedure(
                        receiver,
                        reverse_tools,
                        runtime_names,
                    ),
                }

                task = AgentTask(
                    block_id=block_id,
                    agent="Co-A",
                    action="Pass",
                    flow_index=flow_index,
                    procedure_indexes=(
                        procedure_index,
                        procedure_index + 1,
                    ),
                    element=flow_item["Element"],
                    sub_step=flow_item["Sub-Step"],
                    payload=payload,
                )

                tasks["Co-A"].append(task)
                procedure_to_block[
                    (flow_index, procedure_index)
                ] = block_id
                procedure_to_block[
                    (flow_index, procedure_index + 1)
                ] = block_id
                procedure_index += 2
                continue

            block_id = (
                f"flow_{flow_index:03d}_"
                f"procedure_{human_index:03d}_"
                f"{safe_identifier(procedure.agent)}"
            )

            task = AgentTask(
                block_id=block_id,
                agent=procedure.agent,
                action=procedure.action,
                flow_index=flow_index,
                procedure_indexes=(procedure_index,),
                element=flow_item["Element"],
                sub_step=flow_item["Sub-Step"],
                payload=task_payload_for_procedure(
                    procedure,
                    reverse_tools,
                    runtime_names,
                ),
            )

            tasks[procedure.agent].append(task)
            procedure_to_block[
                (flow_index, procedure_index)
            ] = block_id
            procedure_index += 1

    return tasks, procedure_to_block


def task_to_json(task: AgentTask) -> dict[str, Any]:
    return {
        "block_id": task.block_id,
        "action": task.action,
        "flow_index": task.flow_index,
        "procedure_indexes": [
            index + 1
            for index in task.procedure_indexes
        ],
        "element": task.element,
        "sub_step": task.sub_step,
        **task.payload,
    }


def build_agent_prompt(
    agent_name: str,
    tasks: list[AgentTask],
) -> str:
    return (
        f"Generate all code blocks assigned to {agent_name}.\n\n"
        "Return one block for every task and copy each block_id exactly.\n\n"
        + json.dumps(
            [task_to_json(task) for task in tasks],
            indent=2,
            ensure_ascii=False,
        )
    )


FORBIDDEN_TEXT = (
    "import ",
    "from ",
    "eval(",
    "open(",
    "compile(",
    "__",
    "subprocess",
    "os.",
    "sys.",
    "socket",
)


ALLOWED_METHOD_CALLS = {
    "plan",
    "render_exec",
    "move_to_home",
    "eef_attach",
    "eef_detach",
    "submit",
    "result",
    "shutdown",
}


def repair_unbound_low_level_calls(
    task: AgentTask,
    code_lines: list[str],
) -> list[str]:
    """
    Repair a common LLM formatting error where a low-level controller method is
    emitted as a bare function call, for example:

        eef_detach(...)

    instead of:

        robot_2.eef_detach(...)

    The controller variable is already grounded in the task payload, so this
    repair does not invent any robot, tool, pose, or action.
    """
    if task.agent not in {"Sb-A", "Pb-A"}:
        return code_lines

    controller = task.payload["controller_variable"]
    method_names = (
        "plan",
        "render_exec",
        "move_to_home",
        "eef_attach",
        "eef_detach",
    )

    pattern = re.compile(
        r"^(?P<indent>\s*)(?P<method>"
        + "|".join(method_names)
        + r")\s*\("
    )

    repaired: list[str] = []

    for line in code_lines:
        match = pattern.match(line)

        if match:
            indent_text = match.group("indent")
            method = match.group("method")
            suffix = line[match.end() - 1 :]
            repaired.append(
                f"{indent_text}{controller}.{method}{suffix}"
            )
        else:
            repaired.append(line)

    return repaired


def repair_collaborative_executor_calls(
    task: AgentTask,
    code_lines: list[str],
) -> list[str]:
    """
    Normalize common o3 spellings of ThreadPoolExecutor.

    The generated module imports ThreadPoolExecutor directly, so qualified
    variants such as concurrent.futures.ThreadPoolExecutor are converted to
    the already imported symbol. This changes only the concurrency constructor
    spelling and does not alter robots, tools, poses, or operation order.
    """
    if task.agent != "Co-A":
        return code_lines

    repaired: list[str] = []

    for line in code_lines:
        line = re.sub(
            r"\b(?:[A-Za-z_][A-Za-z0-9_]*\.)+ThreadPoolExecutor\b",
            "ThreadPoolExecutor",
            line,
        )
        repaired.append(line)

    return repaired


def validate_python_block(
    task: AgentTask,
    code_lines: list[str],
) -> list[str]:
    if not isinstance(code_lines, list) or not code_lines:
        raise ValueError(
            f"{task.block_id} returned no code lines."
        )

    if not all(
        isinstance(line, str)
        for line in code_lines
    ):
        raise TypeError(
            f"{task.block_id} contains a non-string code line."
        )

    source = "\n".join(code_lines).strip()

    if not source:
        raise ValueError(
            f"{task.block_id} returned an empty source block."
        )

    lowered = source.casefold()
    for forbidden in FORBIDDEN_TEXT:
        if forbidden.casefold() in lowered:
            raise ValueError(
                f"{task.block_id} contains forbidden text: "
                f"{forbidden!r}"
            )

    wrapped = (
        "def _generated_block("
        "element_object_name, L=0.0, W=0.0, H=0.0, "
        "X=0.0, Y=0.0, Z=0.0, "
        "robot_1=None, robot_2=None, np=None, "
        "ThreadPoolExecutor=None"
        "):\n"
        + "\n".join(
            "    " + line if line else ""
            for line in code_lines
        )
        + "\n"
    )

    tree = ast.parse(wrapped)

    forbidden_nodes = (
        ast.Import,
        ast.ImportFrom,
        ast.FunctionDef,
        ast.AsyncFunctionDef,
        ast.ClassDef,
        ast.Lambda,
        ast.For,
        ast.AsyncFor,
        ast.While,
        ast.Try,
        ast.Raise,
        ast.Delete,
        ast.Global,
        ast.Nonlocal,
        ast.Yield,
        ast.YieldFrom,
        ast.Await,
    )

    # Skip the wrapper function itself.
    for node in ast.walk(tree.body[0]):
        if node is tree.body[0]:
            continue

        if isinstance(node, forbidden_nodes):
            raise ValueError(
                f"{task.block_id} contains unsupported Python syntax: "
                f"{type(node).__name__}"
            )

        if isinstance(node, ast.Attribute):
            if node.attr.startswith("__"):
                raise ValueError(
                    f"{task.block_id} accesses a dunder attribute."
                )

        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Attribute):
                if node.func.attr not in ALLOWED_METHOD_CALLS and not (
                    isinstance(node.func.value, ast.Name)
                    and node.func.value.id == "np"
                    and node.func.attr == "array"
                ):
                    raise ValueError(
                        f"{task.block_id} calls unsupported method "
                        f"{node.func.attr!r}."
                    )
            elif isinstance(node.func, ast.Name):
                if node.func.id != "ThreadPoolExecutor":
                    raise ValueError(
                        f"{task.block_id} calls unsupported function "
                        f"{node.func.id!r}."
                    )

    plan_count = source.count(".plan(")
    exec_count = source.count(".render_exec(")

    if task.agent == "Sb-A":
        if task.action != "Pick":
            raise ValueError("Sb-A received a non-Pick task.")
        if plan_count != 1 or exec_count != 1:
            raise ValueError(
                f"{task.block_id} must have one plan and one render_exec."
            )
        if ".eef_attach(" not in source:
            raise ValueError(
                f"{task.block_id} is missing eef_attach."
            )
        if ".eef_detach(" in source:
            raise ValueError(
                f"{task.block_id} must not detach during Pick."
            )
        if source.count(".move_to_home(") != 1:
            raise ValueError(
                f"{task.block_id} must move home once."
            )

    elif task.agent == "Pb-A":
        if plan_count != 1 or exec_count != 1:
            raise ValueError(
                f"{task.block_id} must have one plan and one render_exec."
            )
        if source.count(".move_to_home(") != 1:
            raise ValueError(
                f"{task.block_id} must move home once."
            )

        if task.action == "Place":
            if ".eef_detach(" not in source:
                raise ValueError(
                    f"{task.block_id} is missing eef_detach."
                )
            if ".eef_attach(" in source:
                raise ValueError(
                    f"{task.block_id} must not attach during Place."
                )
        elif task.action == "Nail":
            if (
                ".eef_attach(" in source
                or ".eef_detach(" in source
            ):
                raise ValueError(
                    f"{task.block_id} must not attach/detach for Nail."
                )
        else:
            raise ValueError(
                f"Pb-A received unsupported action {task.action!r}."
            )

    elif task.agent == "Co-A":
        if plan_count != 2:
            raise ValueError(
                f"{task.block_id} must plan both robots."
            )
        if source.count(".render_exec") != 2:
            raise ValueError(
                f"{task.block_id} must execute both robots."
            )
        if source.count(".eef_attach(") != 1:
            raise ValueError(
                f"{task.block_id} must attach the receiver once."
            )
        if source.count(".eef_detach(") != 1:
            raise ValueError(
                f"{task.block_id} must detach the giver once."
            )
        if source.count(".move_to_home") != 2:
            raise ValueError(
                f"{task.block_id} must send both robots home."
            )
        if "ThreadPoolExecutor" not in source:
            raise ValueError(
                f"{task.block_id} must synchronize with "
                "ThreadPoolExecutor."
            )

    return [
        line.rstrip()
        for line in code_lines
    ]


class O3CodeAgent:
    def __init__(
        self,
        name: str,
        instructions: str,
        client: OpenAI,
        config: dict[str, Any],
    ) -> None:
        self.name = name
        self.instructions = instructions
        self.client = client
        self.config = config

    def generate(
        self,
        tasks: list[AgentTask],
    ) -> tuple[
        dict[str, list[str]],
        list[dict[str, Any]],
        str,
    ]:
        """
        Generate blocks in small batches.

        The previous version sent every Sb-A task in one request. With o3 and
        high reasoning effort, reasoning tokens plus all requested code blocks
        could exhaust max_output_tokens before the complete function call was
        produced. Small batches also make validation feedback specific to the
        failed block(s).
        """
        if not tasks:
            return {}, [], ""

        generation = self.config.get("hlaf_generation", {})

        max_attempts = generation.get(
            "validation_max_attempts",
            self.config.get("agent", {}).get(
                "validation_max_attempts",
                3,
            ),
        )
        if self.name == "Co-A":
            max_attempts = generation.get(
                "co_a_validation_max_attempts",
                max(max_attempts, 5),
            )
        fallback_to_auto = generation.get(
            "fallback_to_auto_tool_choice",
            True,
        )
        batch_size = int(
            generation.get("max_tasks_per_call", 1)
        )
        if batch_size < 1:
            raise ValueError(
                "hlaf_generation.max_tasks_per_call must be at least 1."
            )

        all_validated: dict[str, list[str]] = {}
        all_attempts: list[dict[str, Any]] = []
        prompts: list[str] = []

        batches = [
            tasks[index:index + batch_size]
            for index in range(0, len(tasks), batch_size)
        ]

        for batch_number, batch_tasks in enumerate(
            batches,
            start=1,
        ):
            prompt = build_agent_prompt(
                self.name,
                batch_tasks,
            )
            prompts.append(prompt)

            expected_ids = {
                task.block_id
                for task in batch_tasks
            }
            task_by_id = {
                task.block_id: task
                for task in batch_tasks
            }

            feedback = ""
            last_error = "Unknown generation failure."
            batch_succeeded = False

            for attempt_number in range(
                1,
                max_attempts + 1,
            ):
                attempt_started = time.perf_counter()
                request_prompt = prompt + feedback
                force_tool = True
                response = None

                try:
                    try:
                        response = self.client.responses.create(
                            **request_parameters(
                                self.config,
                                self.instructions,
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
                            response = self.client.responses.create(
                                **request_parameters(
                                    self.config,
                                    self.instructions,
                                    request_prompt
                                    + "\n\nYou must call "
                                    + BLOCK_TOOL_NAME
                                    + " in this response.",
                                    force_tool=False,
                                )
                            )
                        else:
                            raise

                    response_status = getattr(
                        response,
                        "status",
                        None,
                    )
                    incomplete_details = getattr(
                        response,
                        "incomplete_details",
                        None,
                    )

                    if response_status == "incomplete":
                        if hasattr(
                            incomplete_details,
                            "model_dump",
                        ):
                            incomplete_value = (
                                incomplete_details.model_dump(
                                    mode="json"
                                )
                            )
                        else:
                            incomplete_value = str(
                                incomplete_details
                            )

                        raise RuntimeError(
                            "The model response was incomplete: "
                            f"{incomplete_value}. Increase "
                            "hlaf_generation.agent_max_output_tokens "
                            "or lower reasoning_effort."
                        )

                    returned = extract_blocks(response)

                    if set(returned) != expected_ids:
                        missing = sorted(
                            expected_ids - set(returned)
                        )
                        extra = sorted(
                            set(returned) - expected_ids
                        )
                        raise ValueError(
                            f"Missing block ids={missing}; "
                            f"additional block ids={extra}."
                        )

                    repaired_blocks = {
                        block_id: repair_collaborative_executor_calls(
                            task_by_id[block_id],
                            repair_unbound_low_level_calls(
                                task_by_id[block_id],
                                returned[block_id],
                            ),
                        )
                        for block_id in expected_ids
                    }

                    validated = {
                        block_id: validate_python_block(
                            task_by_id[block_id],
                            repaired_blocks[block_id],
                        )
                        for block_id in expected_ids
                    }

                    overlap = (
                        set(all_validated)
                        & set(validated)
                    )
                    if overlap:
                        raise ValueError(
                            "Duplicate blocks generated across "
                            f"batches: {sorted(overlap)}"
                        )

                    all_validated.update(validated)

                    all_attempts.append(
                        {
                            "batch": batch_number,
                            "batch_count": len(batches),
                            "block_ids": sorted(
                                expected_ids
                            ),
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
                            "response_status": response_status,
                            "duration_seconds": round(
                                time.perf_counter()
                                - attempt_started,
                                6,
                            ),
                            "usage": usage_to_dict(
                                response
                            ),
                            "blocks": validated,
                        }
                    )

                    print(
                        f"[{self.name}] Batch "
                        f"{batch_number}/{len(batches)} "
                        f"accepted: "
                        f"{', '.join(sorted(expected_ids))}"
                    )

                    batch_succeeded = True
                    break

                except AuthenticationError:
                    raise

                except Exception as exc:
                    last_error = (
                        f"{type(exc).__name__}: {exc}"
                    )

                    attempt_record: dict[str, Any] = {
                        "batch": batch_number,
                        "batch_count": len(batches),
                        "block_ids": sorted(expected_ids),
                        "attempt": attempt_number,
                        "status": "rejected",
                        "duration_seconds": round(
                            time.perf_counter()
                            - attempt_started,
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
                                "usage": usage_to_dict(
                                    response
                                ),
                            }
                        )

                    all_attempts.append(attempt_record)

                    print(
                        f"[{self.name}] Batch "
                        f"{batch_number}/{len(batches)}, "
                        f"attempt {attempt_number}/"
                        f"{max_attempts} rejected: "
                        f"{last_error}",
                        file=sys.stderr,
                    )

                    feedback = (
                        "\n\nThe previous response was rejected:\n"
                        + last_error
                        + "\nRegenerate every requested block in "
                        "this batch. Copy all block_id values "
                        "exactly and satisfy every agent rule."
                    )

            if not batch_succeeded:
                raise RuntimeError(
                    f"{self.name} failed for batch "
                    f"{batch_number}/{len(batches)} after "
                    f"{max_attempts} attempts. "
                    f"Block ids={sorted(expected_ids)}. "
                    f"Last error: {last_error}"
                )

        return (
            all_validated,
            all_attempts,
            "\n\n--- NEXT BATCH ---\n\n".join(prompts),
        )


def flow_required_variables(
    flow: list[list[Any]],
) -> list[str]:
    discovered: list[str] = []

    for raw_procedure in flow:
        procedure = Procedure.from_json(raw_procedure)

        for variable in procedure.pose.get(
            "required_variables",
            [],
        ):
            if variable not in discovered:
                discovered.append(variable)

    ordered = [
        variable
        for variable in CANONICAL_VARIABLE_ORDER
        if variable in discovered
    ]

    ordered.extend(
        variable
        for variable in discovered
        if variable not in ordered
    )

    return ordered


def flow_controller_bindings(
    flow: list[list[Any]],
) -> list[str]:
    robots: list[str] = []

    for raw_procedure in flow:
        robot = Procedure.from_json(
            raw_procedure
        ).robot

        if robot not in robots:
            robots.append(robot)

    return [
        f'{controller_variable(robot)} = '
        f'self.robots["{robot}"]'
        for robot in robots
    ]


def compose_flow_blocks(
    flow_index: int,
    flow: list[list[Any]],
    procedure_to_block: dict[tuple[int, int], str],
    generated_blocks: dict[str, list[str]],
) -> list[str]:
    lines: list[str] = []
    procedure_index = 0

    while procedure_index < len(flow):
        block_id = procedure_to_block[
            (flow_index, procedure_index)
        ]
        lines.extend(generated_blocks[block_id])
        lines.append("")

        procedure = Procedure.from_json(
            flow[procedure_index]
        )

        if procedure.agent == "Co-A":
            procedure_index += 2
        else:
            procedure_index += 1

    while lines and lines[-1] == "":
        lines.pop()

    return lines


def generate_method(
    flow_index: int,
    flow_item: dict[str, Any],
    procedure_to_block: dict[tuple[int, int], str],
    generated_blocks: dict[str, list[str]],
) -> tuple[list[str], dict[str, Any]]:
    element = flow_item["Element"]
    sub_step = flow_item["Sub-Step"]
    flow = flow_item["HLAF Flow"]

    function_name = (
        f"hlaf_{flow_index:03d}_"
        + safe_identifier(
            f"{element}_{sub_step}",
        )
    )

    variables = flow_required_variables(flow)

    parameters = [
        "self",
        "element_object_name: str",
    ]

    if variables:
        parameters.append("*")
        parameters.extend(
            f"{variable}: float"
            for variable in variables
        )

    signature = (
        f"def {function_name}("
        + ", ".join(parameters)
        + ") -> None:"
    )

    method_lines = [signature]
    method_lines.extend(
        indent(
            [
                '"""',
                f"Element: {element}",
                f"Sub-Step: {sub_step}",
                '"""',
            ]
        )
    )

    body = flow_controller_bindings(flow)
    if body:
        body.append("")

    body.extend(
        compose_flow_blocks(
            flow_index,
            flow,
            procedure_to_block,
            generated_blocks,
        )
    )

    method_lines.extend(indent(body))

    manifest = {
        "index": flow_index,
        "function": function_name,
        "element": element,
        "sub_step": sub_step,
        "required_variables": variables,
        "agents": [
            Procedure.from_json(item).agent
            for item in flow
        ],
        "poses": [
            Procedure.from_json(item).pose["name"]
            for item in flow
        ],
    }

    return method_lines, manifest


def module_header(
    manifest: list[dict[str, Any]],
) -> list[str]:
    return [
        '"""',
        "HLAF methods generated by the o3-based Sb-A, Pb-A, and Co-A agents.",
        "",
        "Example:",
        "    hlafs = GeneratedHLAFs({",
        '        "Robot_1": robot_1_controller,',
        '        "Robot_2": robot_2_controller,',
        "    })",
        '"""',
        "",
        "from __future__ import annotations",
        "",
        "from concurrent.futures import ThreadPoolExecutor",
        "from typing import Any, Mapping",
        "",
        "import numpy as np",
        "",
        "",
        "HLAF_MANIFEST = "
        + json.dumps(
            manifest,
            indent=2,
            ensure_ascii=False,
        ),
        "",
        "",
        "class GeneratedHLAFs:",
        "    def __init__(",
        "        self,",
        "        robots: Mapping[str, Any],",
        "    ) -> None:",
        "        self.robots = dict(robots)",
    ]


def main() -> None:
    started = time.perf_counter()

    record: dict[str, Any] = {
        "generator": "o3 Multi-Agent HLAF Generator",
        "status": "running",
        "started_at_utc": utc_now(),
        "completed_at_utc": None,
        "duration_seconds": None,
        "files": {
            "hlaf_flow_input": FLOW_FILE.name,
            "primitive_definitions": DEFINITIONS_FILE.name,
            "configuration": CONFIG_FILE.name,
            "output": OUTPUT_FILE.name,
            "run_record": RUN_RECORD_FILE.name,
        },
        "agents": {},
    }

    try:
        if not os.getenv("OPENAI_API_KEY"):
            raise RuntimeError(
                "OPENAI_API_KEY is not set."
            )

        flow_data = read_json(FLOW_FILE)
        definitions = read_json(DEFINITIONS_FILE)
        config = read_json(CONFIG_FILE)

        model = get_model_profile(config)
        api = config.get("api", {})
        runtime_names = {
            **DEFAULT_RUNTIME_NAMES,
            **config.get("robot_runtime_names", {}),
        }

        reverse_tools = reverse_tool_map(definitions)
        flows = flow_data["HLAF-Flows"]

        tasks_by_agent, procedure_to_block = build_tasks(
            flows,
            reverse_tools,
            runtime_names,
        )

        record.update(
            {
                "input_hashes_sha256": {
                    FLOW_FILE.name: sha256_file(FLOW_FILE),
                    DEFINITIONS_FILE.name: sha256_file(
                        DEFINITIONS_FILE
                    ),
                    CONFIG_FILE.name: sha256_file(CONFIG_FILE),
                },
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
                    "max_output_tokens": config.get(
                        "hlaf_generation",
                        {},
                    ).get(
                        "agent_max_output_tokens",
                        model.get(
                            "max_output_tokens",
                            4096,
                        ),
                    ),
                    "sdk_max_retries": api.get(
                        "sdk_max_retries",
                        2,
                    ),
                    "validation_max_attempts": config.get(
                        "hlaf_generation",
                        {},
                    ).get(
                        "validation_max_attempts",
                        config.get("agent", {}).get(
                            "validation_max_attempts",
                            3,
                        ),
                    ),
                    "strict_tool_schema": True,
                    "parallel_tool_calls": False,
                    "fallback_to_auto_tool_choice": config.get(
                        "hlaf_generation",
                        {},
                    ).get(
                        "fallback_to_auto_tool_choice",
                        True,
                    ),
                    "reuse_accepted_agent_blocks": config.get(
                        "hlaf_generation",
                        {},
                    ).get(
                        "reuse_accepted_agent_blocks",
                        True,
                    ),
                    "co_a_validation_max_attempts": config.get(
                        "hlaf_generation",
                        {},
                    ).get(
                        "co_a_validation_max_attempts",
                        5,
                    ),
                },
                "runtime_robot_names": runtime_names,
                "software": {
                    "python_version": platform.python_version(),
                    "openai_python_version": (
                        importlib.metadata.version("openai")
                    ),
                },
                "agent_task_counts": {
                    agent: len(tasks)
                    for agent, tasks in tasks_by_agent.items()
                },
                "tool_schema": BLOCK_TOOL,
            }
        )

        client = create_client(config)

        agents = {
            "Sb-A": O3CodeAgent(
                "Sb-A",
                SB_A_INSTRUCTIONS,
                client,
                config,
            ),
            "Pb-A": O3CodeAgent(
                "Pb-A",
                PB_A_INSTRUCTIONS,
                client,
                config,
            ),
            "Co-A": O3CodeAgent(
                "Co-A",
                CO_A_INSTRUCTIONS,
                client,
                config,
            ),
        }

        all_blocks: dict[str, list[str]] = {}

        previous_record: dict[str, Any] = {}
        reuse_enabled = config.get(
            "hlaf_generation",
            {},
        ).get("reuse_accepted_agent_blocks", True)

        if reuse_enabled and RUN_RECORD_FILE.exists():
            try:
                candidate_record = read_json(RUN_RECORD_FILE)
                candidate_hashes = candidate_record.get(
                    "input_hashes_sha256",
                    {},
                )
                current_hashes = record[
                    "input_hashes_sha256"
                ]

                same_inputs = (
                    candidate_hashes.get(FLOW_FILE.name)
                    == current_hashes.get(FLOW_FILE.name)
                    and candidate_hashes.get(
                        DEFINITIONS_FILE.name
                    )
                    == current_hashes.get(
                        DEFINITIONS_FILE.name
                    )
                    and candidate_record.get(
                        "runtime_robot_names"
                    )
                    == runtime_names
                )

                if same_inputs:
                    previous_record = candidate_record
            except Exception:
                previous_record = {}

        for agent_name in ("Sb-A", "Pb-A", "Co-A"):
            agent_tasks = tasks_by_agent[agent_name]
            task_by_id = {
                task.block_id: task
                for task in agent_tasks
            }
            expected_ids = set(task_by_id)

            previous_agent = previous_record.get(
                "agents",
                {},
            ).get(agent_name, {})
            previous_blocks = previous_agent.get(
                "generated_blocks",
                {},
            )

            reused = False
            generated: dict[str, list[str]]
            attempts: list[dict[str, Any]]
            prompt: str

            if (
                expected_ids
                and set(previous_blocks) == expected_ids
            ):
                try:
                    generated = {
                        block_id: validate_python_block(
                            task_by_id[block_id],
                            repair_collaborative_executor_calls(
                                task_by_id[block_id],
                                repair_unbound_low_level_calls(
                                    task_by_id[block_id],
                                    previous_blocks[block_id],
                                ),
                            ),
                        )
                        for block_id in expected_ids
                    }
                    attempts = [
                        {
                            "status": "reused_from_previous_run",
                            "block_ids": sorted(expected_ids),
                        }
                    ]
                    prompt = previous_agent.get(
                        "prompt",
                        "",
                    )
                    reused = True
                    print(
                        f"[{agent_name}] Reused "
                        f"{len(generated)} accepted blocks "
                        "from the previous run record."
                    )
                except Exception:
                    reused = False

            if not reused:
                generated, attempts, prompt = agents[
                    agent_name
                ].generate(agent_tasks)

            overlap = set(all_blocks) & set(generated)
            if overlap:
                raise ValueError(
                    f"Duplicate block ids across agents: "
                    f"{sorted(overlap)}"
                )

            all_blocks.update(generated)

            instructions = {
                "Sb-A": SB_A_INSTRUCTIONS,
                "Pb-A": PB_A_INSTRUCTIONS,
                "Co-A": CO_A_INSTRUCTIONS,
            }[agent_name]

            record["agents"][agent_name] = {
                "instructions": instructions,
                "instructions_sha256": sha256_text(
                    instructions
                ),
                "prompt": prompt,
                "prompt_sha256": (
                    sha256_text(prompt)
                    if prompt
                    else None
                ),
                "tasks": [
                    task_to_json(task)
                    for task in agent_tasks
                ],
                "attempts": attempts,
                "generated_blocks": generated,
            }

        methods: list[list[str]] = []
        manifest: list[dict[str, Any]] = []

        for flow_index, flow_item in enumerate(
            flows,
            start=1,
        ):
            method, manifest_item = generate_method(
                flow_index,
                flow_item,
                procedure_to_block,
                all_blocks,
            )
            methods.append(method)
            manifest.append(manifest_item)

        source_lines = module_header(manifest)

        for method in methods:
            source_lines.append("")
            source_lines.extend(indent(method))

        source_lines.append("")
        generated_source = "\n".join(source_lines)

        # Final whole-file syntax validation.
        compile(
            generated_source,
            str(OUTPUT_FILE),
            "exec",
        )

        OUTPUT_FILE.write_text(
            generated_source + "\n",
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
                "generated_hlaf_count": len(manifest),
                "generated_block_count": len(all_blocks),
                "manifest": manifest,
                "output_sha256": sha256_file(OUTPUT_FILE),
            }
        )
        write_json(RUN_RECORD_FILE, record)

        print(f"Generated: {OUTPUT_FILE}")
        print(f"Generated: {RUN_RECORD_FILE}")
        print(f"HLAF methods: {len(manifest)}")
        print(
            "Agent calls: "
            + ", ".join(
                f"{name}={1 if tasks_by_agent[name] else 0}"
                for name in ("Sb-A", "Pb-A", "Co-A")
            )
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
        write_json(RUN_RECORD_FILE, record)
        raise


if __name__ == "__main__":
    main()
