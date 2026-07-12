#!/usr/bin/env python3
"""
L-A-II: select robotically executable Sub-Steps and generate their HLAF flows.

Place these files in the same directory as this script:
    L_A_I_Output.json
    Primitive_Library_Definitions.json
    Primitive_Library_Poses.json
    Primitive_Library_Relations.json
        or Primitive_Library_Relations(1).json
    Agent_Description.json
        or Agent_Description(1).json
    config.json

Required environment variable:
    OPENAI_API_KEY

Created files:
    L_A_II_Output.json
    L_A_II_Run_Record.json
"""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import os
import platform
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from openai import OpenAI


ROOT = Path(__file__).resolve().parent

L_A_I_FILE = ROOT / "L_A_I_Output.json"
DEFINITIONS_FILE = ROOT / "Primitive_Library_Definitions.json"
POSES_FILE = ROOT / "Primitive_Library_Poses.json"
CONFIG_FILE = ROOT / "config.json"

OUTPUT_FILE = ROOT / "L_A_II_Output.json"
RUN_RECORD_FILE = ROOT / "L_A_II_Run_Record.json"

TOOL_NAME = "select_robotic_sub_steps"

CAPABILITIES = (
    "PickOnly",
    "TransportAndPlace",
    "Nail",
)

# These aliases resolve naming differences between L-A-I and the primitive
# relation library.
ELEMENT_ALIASES = {
    "Bear Loading": "Load_Bearing",
    "Lower Cripple Stud": "Lower_Cripple_Plate",
    "Top Cripple Stud": "Top_Cripple_Plate",
}


SELECTION_TOOL: dict[str, Any] = {
    "type": "function",
    "name": TOOL_NAME,
    "description": (
        "Select only the L-A-I Sub-Steps that can be accomplished using "
        "the available robotic primitive capabilities."
    ),
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "Selections": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "Element": {
                            "type": "string",
                            "description": (
                                "Exact element name copied from L_A_I_Output.json."
                            ),
                        },
                        "SubStep": {
                            "type": "string",
                            "description": (
                                "Exact Sub-Step text copied from "
                                "L_A_I_Output.json."
                            ),
                        },
                        "Capability": {
                            "type": "string",
                            "enum": list(CAPABILITIES),
                            "description": (
                                "The primitive capability that can accomplish "
                                "the selected Sub-Step."
                            ),
                        },
                    },
                    "required": [
                        "Element",
                        "SubStep",
                        "Capability",
                    ],
                    "additionalProperties": False,
                },
            }
        },
        "required": ["Selections"],
        "additionalProperties": False,
    },
}


SYSTEM_PROMPT = """
You are L-A-II in a construction robotic-planning pipeline.

Select only the L-A-I Sub-Steps that are directly executable using the
available robotic primitive capabilities supplied in the prompt.

Capability meanings:
- PickOnly: the Sub-Step explicitly requires only picking or grasping the
  component, without placing it.
- TransportAndPlace: the Sub-Step requires lifting, setting, standing,
  positioning, placing, or installing the component. The application will
  automatically expand this into Pick -> optional collaborative Pass -> Place.
- Nail: the Sub-Step requires fastening with nails and a Nail primitive exists.

Rules:
- Copy the Element and SubStep text exactly from the supplied L-A-I data.
- Select only capabilities listed as available for that element.
- Select at most one TransportAndPlace Sub-Step and one Nail Sub-Step for each
  element.
- Select PickOnly only when the Sub-Step explicitly describes picking without
  placement.
- A fastening Sub-Step may be selected as Nail when nailing is one of the
  stated fastening methods and the element has a Nail capability.
- Ignore planning, reviewing, measuring, marking, cutting, drilling, checking,
  alignment-only, leveling-only, bracing, reporting, scheduling, estimating,
  repair, gluing, and other unsupported work.
- Do not select a Sub-Step merely because it mentions a tool; the physical
  objective must match an available capability.
- Do not invent elements, Sub-Steps, capabilities, robots, tools, poses, or
  agents.
- Return the selections only through the select_robotic_sub_steps function.
""".strip()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def save_json(path: Path, data: Any) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2, ensure_ascii=False)
        file.write("\n")


def find_file(patterns: list[str]) -> Path:
    for pattern in patterns:
        exact = ROOT / pattern
        if exact.exists():
            return exact

    matches: list[Path] = []
    for pattern in patterns:
        matches.extend(ROOT.glob(pattern))

    unique_matches = sorted(set(matches), key=lambda path: path.name)
    if not unique_matches:
        raise FileNotFoundError(
            "Could not find any of: " + ", ".join(patterns)
        )
    return unique_matches[0]


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def usage_to_dict(response: Any) -> dict[str, Any] | None:
    usage = getattr(response, "usage", None)
    if hasattr(usage, "model_dump"):
        return usage.model_dump(mode="json")
    return None


def normalize_element_name(name: str) -> str:
    return re.sub(r"[^a-z0-9]", "", name.casefold())


def pose_action(pose_name: str) -> str | None:
    lowered = pose_name.casefold()
    if "_pick_pose" in lowered:
        return "Pick"
    if "_place_pose" in lowered:
        return "Place"
    if "_pass_pose" in lowered:
        return "Pass"
    if "_nail_pose" in lowered:
        return "Nail"
    return None


def get_model_profile(config: dict[str, Any]) -> dict[str, Any]:
    profile_name = config["active_model_profile"]
    return config["model_profiles"][profile_name]


def load_config() -> dict[str, Any]:
    if CONFIG_FILE.exists():
        return load_json(CONFIG_FILE)

    # Fallback allows the script to run when the prior shared config has not
    # been copied into the directory.
    return {
        "active_model_profile": "o3",
        "api": {
            "api_family": "Responses API",
            "api_version": "v1",
            "base_url": None,
            "timeout_seconds": 120,
            "sdk_max_retries": 2,
        },
        "model_profiles": {
            "o3": {
                "identifier": "o3",
                "reasoning_effort": "high",
                "temperature": None,
                "top_p": None,
                "max_output_tokens": 4096,
            }
        },
        "request": {"store": False},
        "agent": {"validation_max_attempts": 3},
    }


def create_client(config: dict[str, Any]) -> OpenAI:
    api_config = config.get("api", {})

    options: dict[str, Any] = {
        "api_key": os.environ["OPENAI_API_KEY"],
        "timeout": api_config.get("timeout_seconds", 120),
        "max_retries": api_config.get("sdk_max_retries", 2),
    }

    base_url = os.getenv("OPENAI_BASE_URL") or api_config.get("base_url")
    if base_url:
        options["base_url"] = base_url

    return OpenAI(**options)


def build_request(
    config: dict[str, Any],
    prompt: str,
) -> dict[str, Any]:
    model = get_model_profile(config)

    request: dict[str, Any] = {
        "model": model["identifier"],
        "instructions": SYSTEM_PROMPT,
        "input": prompt,
        "tools": [SELECTION_TOOL],
        "tool_choice": {
            "type": "function",
            "name": TOOL_NAME,
        },
        "parallel_tool_calls": False,
        "max_output_tokens": model.get("max_output_tokens", 4096),
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


def extract_tool_arguments(response: Any) -> dict[str, Any]:
    for item in response.output:
        if item.type == "function_call" and item.name == TOOL_NAME:
            return json.loads(item.arguments)

    raise RuntimeError(f"The model did not call {TOOL_NAME}.")


def resolve_agent_names(
    descriptions: dict[str, str],
) -> dict[str, str]:
    resolved: dict[str, str] = {}

    for name in descriptions:
        lowered = name.casefold()
        if "simulation" in lowered or "sb-a" in lowered:
            resolved["Pick"] = name
        elif "parameter" in lowered or "pb-a" in lowered:
            resolved["Place"] = name
            resolved["Nail"] = name
        elif "collaborative" in lowered or "co-a" in lowered:
            resolved["Pass"] = name

    required = {"Pick", "Place", "Nail", "Pass"}
    missing = required - set(resolved)
    if missing:
        raise ValueError(
            "Agent descriptions do not define agents for: "
            + ", ".join(sorted(missing))
        )

    return resolved


def relation_element_for(
    l_a_i_element: str,
    relation_elements: list[str],
) -> str | None:
    alias = ELEMENT_ALIASES.get(l_a_i_element)
    if alias and alias in relation_elements:
        return alias

    normalized_target = normalize_element_name(l_a_i_element)

    for relation_element in relation_elements:
        if normalize_element_name(relation_element) == normalized_target:
            return relation_element

    return None


def build_relation_index(
    relations: list[dict[str, Any]],
) -> dict[str, dict[str, list[dict[str, Any]]]]:
    index: dict[str, dict[str, list[dict[str, Any]]]] = {}

    for relation in relations:
        element = relation["element"]
        action = pose_action(relation["pose"])
        if action is None:
            continue

        index.setdefault(element, {}).setdefault(action, []).append(relation)

    return index


def singular_robot_and_tool(
    relation: dict[str, Any],
) -> tuple[str, str]:
    return relation["robot"], relation["tool_id"]


def find_transfer_relation(
    pass_relations: list[dict[str, Any]],
    source_robot: str,
    source_tool: str,
    target_robot: str,
    target_tool: str,
) -> dict[str, Any] | None:
    for relation in pass_relations:
        if (
            relation.get("robot_1") == source_robot
            and relation.get("tool_id_1") == source_tool
            and relation.get("robot_2") == target_robot
            and relation.get("tool_id_2") == target_tool
        ):
            return relation
    return None


def available_capabilities(
    relation_index: dict[str, dict[str, list[dict[str, Any]]]],
) -> dict[str, dict[str, Any]]:
    capabilities: dict[str, dict[str, Any]] = {}

    for element, actions in relation_index.items():
        pick_relations = actions.get("Pick", [])
        place_relations = actions.get("Place", [])
        pass_relations = actions.get("Pass", [])

        transport_available = False
        requires_pass = False

        if pick_relations and place_relations:
            pick_robot, pick_tool = singular_robot_and_tool(
                pick_relations[0]
            )
            place_robot, place_tool = singular_robot_and_tool(
                place_relations[0]
            )

            if (pick_robot, pick_tool) == (place_robot, place_tool):
                transport_available = True
            else:
                bridge = find_transfer_relation(
                    pass_relations,
                    pick_robot,
                    pick_tool,
                    place_robot,
                    place_tool,
                )
                transport_available = bridge is not None
                requires_pass = bridge is not None

        capabilities[element] = {
            "PickOnly": bool(pick_relations),
            "TransportAndPlace": transport_available,
            "TransportRequiresPass": requires_pass,
            "Nail": bool(actions.get("Nail", [])),
        }

    return capabilities


def validate_selections(
    arguments: dict[str, Any],
    substeps_by_element: dict[str, list[str]],
    relation_element_map: dict[str, str],
    capabilities: dict[str, dict[str, Any]],
) -> list[dict[str, str]]:
    selections = arguments.get("Selections", [])
    if not isinstance(selections, list) or not selections:
        raise ValueError("No robotically executable Sub-Step was selected.")

    validated: list[dict[str, str]] = []
    seen: set[tuple[str, str]] = set()
    per_element_capability: set[tuple[str, str]] = set()

    for selection in selections:
        element = selection.get("Element")
        substep = selection.get("SubStep")
        capability = selection.get("Capability")

        if element not in substeps_by_element:
            raise ValueError(f"Unknown element selected: {element!r}")

        if substep not in substeps_by_element[element]:
            raise ValueError(
                f"Sub-Step was not copied exactly for {element!r}: "
                f"{substep!r}"
            )

        if capability not in CAPABILITIES:
            raise ValueError(f"Unsupported capability: {capability!r}")

        relation_element = relation_element_map.get(element)
        if relation_element is None:
            raise ValueError(
                f"No primitive-library element matches {element!r}."
            )

        if not capabilities[relation_element].get(capability, False):
            raise ValueError(
                f"{capability} is unavailable for element {element!r}."
            )

        exact_key = (element, substep)
        if exact_key in seen:
            raise ValueError(
                f"Duplicate selection for {element!r}: {substep!r}"
            )

        action_key = (element, capability)
        if action_key in per_element_capability:
            raise ValueError(
                f"More than one {capability} Sub-Step selected for "
                f"{element!r}."
            )

        seen.add(exact_key)
        per_element_capability.add(action_key)
        validated.append(
            {
                "Element": element,
                "SubStep": substep,
                "Capability": capability,
            }
        )

    return validated


def pose_payload(
    pose_name: str,
    pose_library: dict[str, Any],
) -> dict[str, Any]:
    pose_definition = pose_library["poses"].get(pose_name)
    if pose_definition is None:
        raise KeyError(f"Pose definition not found: {pose_name}")

    return {
        "name": pose_name,
        "units": pose_library.get("units", {}),
        **pose_definition,
    }


def tool_name(
    definitions: dict[str, dict[str, str]],
    robot: str,
    tool_id: str,
) -> str:
    try:
        return definitions[robot][tool_id]
    except KeyError as exc:
        raise KeyError(
            f"Tool definition not found for {robot}.{tool_id}"
        ) from exc


def singular_procedure(
    relation: dict[str, Any],
    display_element: str,
    action: str,
    definitions: dict[str, dict[str, str]],
    pose_library: dict[str, Any],
    agents: dict[str, str],
) -> list[Any]:
    robot = relation["robot"]
    tool_id = relation["tool_id"]

    return [
        robot,
        tool_name(definitions, robot, tool_id),
        display_element,
        pose_payload(relation["pose"], pose_library),
        agents[action],
    ]


def collaborative_pass_procedures(
    relation: dict[str, Any],
    display_element: str,
    definitions: dict[str, dict[str, str]],
    pose_library: dict[str, Any],
    agents: dict[str, str],
) -> list[list[Any]]:
    pose = pose_payload(relation["pose"], pose_library)
    procedures: list[list[Any]] = []

    for robot_key, tool_key in (
        ("robot_1", "tool_id_1"),
        ("robot_2", "tool_id_2"),
    ):
        robot = relation[robot_key]
        tool_id = relation[tool_key]

        procedures.append(
            [
                robot,
                tool_name(definitions, robot, tool_id),
                display_element,
                pose,
                agents["Pass"],
            ]
        )

    return procedures


def build_hlaf_flow(
    selection: dict[str, str],
    relation_element: str,
    relation_index: dict[str, dict[str, list[dict[str, Any]]]],
    definitions: dict[str, dict[str, str]],
    pose_library: dict[str, Any],
    agents: dict[str, str],
) -> list[list[Any]]:
    element = selection["Element"]
    capability = selection["Capability"]
    actions = relation_index[relation_element]

    if capability == "PickOnly":
        return [
            singular_procedure(
                actions["Pick"][0],
                element,
                "Pick",
                definitions,
                pose_library,
                agents,
            )
        ]

    if capability == "Nail":
        return [
            singular_procedure(
                actions["Nail"][0],
                element,
                "Nail",
                definitions,
                pose_library,
                agents,
            )
        ]

    if capability == "TransportAndPlace":
        pick_relation = actions["Pick"][0]
        place_relation = actions["Place"][0]

        flow = [
            singular_procedure(
                pick_relation,
                element,
                "Pick",
                definitions,
                pose_library,
                agents,
            )
        ]

        pick_robot, pick_tool = singular_robot_and_tool(
            pick_relation
        )
        place_robot, place_tool = singular_robot_and_tool(
            place_relation
        )

        if (pick_robot, pick_tool) != (place_robot, place_tool):
            transfer = find_transfer_relation(
                actions.get("Pass", []),
                pick_robot,
                pick_tool,
                place_robot,
                place_tool,
            )
            if transfer is None:
                raise RuntimeError(
                    f"No collaborative pass connects the Pick and Place "
                    f"relations for {element!r}."
                )

            flow.extend(
                collaborative_pass_procedures(
                    transfer,
                    element,
                    definitions,
                    pose_library,
                    agents,
                )
            )

        flow.append(
            singular_procedure(
                place_relation,
                element,
                "Place",
                definitions,
                pose_library,
                agents,
            )
        )
        return flow

    raise ValueError(f"Unhandled capability: {capability}")


def main() -> None:
    started = time.perf_counter()

    relations_file = find_file(
        [
            "Primitive_Library_Relations.json",
            "Primitive_Library_Relations(1).json",
            "Primitive_Library_Relations*.json",
        ]
    )
    agent_file = find_file(
        [
            "Agent_Description.json",
            "Agent_Description(1).json",
            "Agent_Description*.json",
        ]
    )

    record: dict[str, Any] = {
        "agent": "L-A-II",
        "status": "running",
        "started_at_utc": utc_now(),
        "completed_at_utc": None,
        "duration_seconds": None,
        "files": {
            "l_a_i_input": L_A_I_FILE.name,
            "primitive_definitions": DEFINITIONS_FILE.name,
            "primitive_poses": POSES_FILE.name,
            "primitive_relations": relations_file.name,
            "agent_descriptions": agent_file.name,
            "configuration": (
                CONFIG_FILE.name
                if CONFIG_FILE.exists()
                else "built-in fallback"
            ),
            "output": OUTPUT_FILE.name,
            "run_record": RUN_RECORD_FILE.name,
        },
        "attempts": [],
    }

    try:
        if not os.getenv("OPENAI_API_KEY"):
            raise RuntimeError("OPENAI_API_KEY is not set.")

        l_a_i = load_json(L_A_I_FILE)
        definitions = load_json(DEFINITIONS_FILE)
        pose_library = load_json(POSES_FILE)
        relations = load_json(relations_file)
        agent_descriptions = load_json(agent_file)
        config = load_config()

        substeps_by_element = {
            element: steps
            for element, steps in l_a_i["Sub-Steps"]
        }

        relation_index = build_relation_index(relations)
        relation_elements = list(relation_index)

        relation_element_map: dict[str, str] = {}
        for element in substeps_by_element:
            matched = relation_element_for(
                element,
                relation_elements,
            )
            if matched is not None:
                relation_element_map[element] = matched

        capability_index = available_capabilities(relation_index)
        agents = resolve_agent_names(agent_descriptions)

        llm_input: list[dict[str, Any]] = []

        for element, steps in substeps_by_element.items():
            relation_element = relation_element_map.get(element)
            if relation_element is None:
                available = {
                    "PickOnly": False,
                    "TransportAndPlace": False,
                    "Nail": False,
                }
            else:
                available = {
                    name: capability_index[relation_element][name]
                    for name in CAPABILITIES
                }

            llm_input.append(
                {
                    "Element": element,
                    "SubSteps": steps,
                    "AvailableCapabilities": available,
                }
            )

        prompt = (
            "Select only robotically executable Sub-Steps from the data "
            "below.\n\n"
            + json.dumps(llm_input, indent=2, ensure_ascii=False)
        )

        model = get_model_profile(config)
        api_config = config.get("api", {})
        max_attempts = config.get("agent", {}).get(
            "validation_max_attempts",
            3,
        )

        record.update(
            {
                "input_hashes_sha256": {
                    L_A_I_FILE.name: sha256_file(L_A_I_FILE),
                    DEFINITIONS_FILE.name: sha256_file(
                        DEFINITIONS_FILE
                    ),
                    POSES_FILE.name: sha256_file(POSES_FILE),
                    relations_file.name: sha256_file(relations_file),
                    agent_file.name: sha256_file(agent_file),
                    **(
                        {
                            CONFIG_FILE.name: sha256_file(CONFIG_FILE)
                        }
                        if CONFIG_FILE.exists()
                        else {}
                    ),
                },
                "relation_element_map": relation_element_map,
                "available_capabilities": {
                    element: capability_index[
                        relation_element_map[element]
                    ]
                    for element in relation_element_map
                },
                "agent_assignments": agents,
                "configuration": {
                    "active_model_profile": config[
                        "active_model_profile"
                    ],
                    "requested_model_identifier": model["identifier"],
                    "api_family": api_config.get("api_family"),
                    "api_version": api_config.get("api_version"),
                    "base_url": (
                        os.getenv("OPENAI_BASE_URL")
                        or api_config.get("base_url")
                        or "OpenAI default"
                    ),
                    "reasoning_effort": model.get(
                        "reasoning_effort"
                    ),
                    "temperature": model.get("temperature"),
                    "top_p": model.get("top_p"),
                    "max_output_tokens": model.get(
                        "max_output_tokens",
                        4096,
                    ),
                    "sdk_max_retries": api_config.get(
                        "sdk_max_retries",
                        2,
                    ),
                    "validation_max_attempts": max_attempts,
                    "parallel_tool_calls": False,
                    "strict_tool_schema": True,
                    "tool_choice": TOOL_NAME,
                },
                "software": {
                    "python_version": platform.python_version(),
                    "openai_python_version": (
                        importlib.metadata.version("openai")
                    ),
                },
                "prompt_record": {
                    "system_prompt": SYSTEM_PROMPT,
                    "system_prompt_sha256": sha256_text(
                        SYSTEM_PROMPT
                    ),
                    "user_prompt": prompt,
                    "user_prompt_sha256": sha256_text(prompt),
                },
                "selection_tool_schema": SELECTION_TOOL,
            }
        )

        client = create_client(config)
        last_error = "Unknown selection failure."
        accepted_selections: list[dict[str, str]] | None = None

        feedback = ""

        for attempt_number in range(1, max_attempts + 1):
            attempt_started = time.perf_counter()
            request_prompt = prompt + feedback

            try:
                response = client.responses.create(
                    **build_request(config, request_prompt)
                )
                arguments = extract_tool_arguments(response)
                selections = validate_selections(
                    arguments,
                    substeps_by_element,
                    relation_element_map,
                    capability_index,
                )

                record["attempts"].append(
                    {
                        "attempt": attempt_number,
                        "status": "accepted",
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
                            time.perf_counter() - attempt_started,
                            6,
                        ),
                        "usage": usage_to_dict(response),
                        "tool_arguments": arguments,
                        "validated_selections": selections,
                    }
                )

                accepted_selections = selections
                break

            except Exception as exc:
                last_error = f"{type(exc).__name__}: {exc}"

                record["attempts"].append(
                    {
                        "attempt": attempt_number,
                        "status": "rejected",
                        "duration_seconds": round(
                            time.perf_counter() - attempt_started,
                            6,
                        ),
                        "error": last_error,
                    }
                )

                feedback = (
                    "\n\nThe previous selection was rejected for this "
                    f"reason:\n{last_error}\n"
                    "Return a corrected selection using only the exact "
                    "elements, exact Sub-Steps, and available capabilities."
                )

        if accepted_selections is None:
            raise RuntimeError(
                f"L-A-II selection failed after {max_attempts} attempts. "
                f"Last error: {last_error}"
            )

        hlaf_flows: list[dict[str, Any]] = []

        for selection in accepted_selections:
            element = selection["Element"]
            relation_element = relation_element_map[element]

            flow = build_hlaf_flow(
                selection,
                relation_element,
                relation_index,
                definitions,
                pose_library,
                agents,
            )

            if not flow:
                raise RuntimeError(
                    f"No HLAF procedure was generated for "
                    f"{selection['SubStep']!r}."
                )

            hlaf_flows.append(
                {
                    "Element": element,
                    "Sub-Step": selection["SubStep"],
                    "HLAF Flow": flow,
                }
            )

        selected_pairs = {
            (selection["Element"], selection["SubStep"])
            for selection in accepted_selections
        }
        ignored_substeps = [
            [element, substep]
            for element, steps in substeps_by_element.items()
            for substep in steps
            if (element, substep) not in selected_pairs
        ]

        final_output = {
            "HLAF-Flows": hlaf_flows
        }
        save_json(OUTPUT_FILE, final_output)

        record.update(
            {
                "status": "success",
                "completed_at_utc": utc_now(),
                "duration_seconds": round(
                    time.perf_counter() - started,
                    6,
                ),
                "selected_sub_steps": accepted_selections,
                "ignored_sub_steps": ignored_substeps,
                "final_output": final_output,
            }
        )
        save_json(RUN_RECORD_FILE, record)

        print(
            json.dumps(
                final_output,
                indent=2,
                ensure_ascii=False,
            )
        )
        print(f"\nSaved: {OUTPUT_FILE}")
        print(f"Saved: {RUN_RECORD_FILE}")

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
        save_json(RUN_RECORD_FILE, record)
        raise


if __name__ == "__main__":
    main()
