#!/usr/bin/env python3
"""
Ext-A-II only.

Expected files in the same directory as this script:
    Material.json
    Ext_A_I_Output.json
    config.json

Created files:
    Ext_A_II_Output.json
    Ext_A_II_Run_Record.json
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


BASE_DIR = Path(__file__).resolve().parent

MATERIAL_FILE = BASE_DIR / "Material.json"
EXT_A_I_FILE = BASE_DIR / "Ext_A_I_Output.json"
CONFIG_FILE = BASE_DIR / "config.json"

OUTPUT_FILE = BASE_DIR / "Ext_A_II_Output.json"
RUN_RECORD_FILE = BASE_DIR / "Ext_A_II_Run_Record.json"

TOOL_NAME = "submit_ext_a_ii_crew_mapping"


EXT_A_II_TOOL: dict[str, Any] = {
    "type": "function",
    "name": TOOL_NAME,
    "description": (
        "Submit the human crew types required to install every provided "
        "assembly element."
    ),
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "Crews": {
                "type": "array",
                "description": (
                    "A crew mapping for every supplied assembly element."
                ),
                "items": {
                    "type": "object",
                    "properties": {
                        "Element": {
                            "type": "string",
                            "description": (
                                "The exact element name copied from "
                                "Material.json."
                            ),
                        },
                        "RequiredCrews": {
                            "type": "array",
                            "description": (
                                "One or more human crew or trade labels "
                                "required to install the element."
                            ),
                            "items": {
                                "type": "string"
                            },
                        },
                    },
                    "required": ["Element", "RequiredCrews"],
                    "additionalProperties": False,
                },
            }
        },
        "required": ["Crews"],
        "additionalProperties": False,
    },
}


SYSTEM_PROMPT = """
You are Ext-A-II in a construction knowledge-extraction pipeline.

Your role is to determine the human crew types required to install each
provided assembly element. Use:

1. The Element Definitions from Material.json.
2. The Structure identified by Ext-A-I.
3. The Material identified by Ext-A-I.
4. Established construction crew and trade classifications consistent with
   RSMeans-style construction activity organization.

Requirements:
- Return exactly one mapping for every supplied element.
- Copy each element name exactly as provided.
- Every RequiredCrews list must contain at least one valid human crew or trade.
- Return crew or trade labels, not equipment, materials, or installation steps.
- Do not add elements that were not supplied.
- Do not claim or invent exact RSMeans crew codes when no RSMeans reference
  table has been supplied.
- Return the result only through the submit_ext_a_ii_crew_mapping tool.
""".strip()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def write_json(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2, ensure_ascii=False)
        file.write("\n")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def usage_to_dict(response: Any) -> dict[str, Any] | None:
    usage = getattr(response, "usage", None)
    if usage is None:
        return None
    if hasattr(usage, "model_dump"):
        return usage.model_dump(mode="json")
    return None


def get_selected_model(config: dict[str, Any]) -> dict[str, Any]:
    active_profile = config["active_model_profile"]
    return config["model_profiles"][active_profile]


def create_client(config: dict[str, Any]) -> OpenAI:
    api_config = config.get("api", {})

    client_options: dict[str, Any] = {
        "api_key": os.environ["OPENAI_API_KEY"],
        "timeout": api_config.get("timeout_seconds", 120),
        "max_retries": api_config.get("sdk_max_retries", 2),
    }

    base_url = os.getenv("OPENAI_BASE_URL") or api_config.get("base_url")
    if base_url:
        client_options["base_url"] = base_url

    return OpenAI(**client_options)


def build_request(
    config: dict[str, Any],
    user_prompt: str,
) -> dict[str, Any]:
    model = get_selected_model(config)

    request: dict[str, Any] = {
        "model": model["identifier"],
        "instructions": SYSTEM_PROMPT,
        "input": user_prompt,
        "tools": [EXT_A_II_TOOL],
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


def validate_and_format_crews(
    tool_arguments: dict[str, Any],
    expected_elements: list[str],
) -> dict[str, Any]:
    raw_crews = tool_arguments.get("Crews", [])

    crew_by_element: dict[str, list[str]] = {}

    for mapping in raw_crews:
        element = mapping.get("Element")
        crews = mapping.get("RequiredCrews")

        if (
            not isinstance(element, str)
            or not isinstance(crews, list)
            or not crews
            or not all(
                isinstance(crew, str) and crew.strip()
                for crew in crews
            )
        ):
            raise ValueError(
                "Each element must have a non-empty list of crew labels."
            )

        if element in crew_by_element:
            raise ValueError(
                f"Duplicate crew mapping returned for: {element}"
            )

        crew_by_element[element] = [
            crew.strip() for crew in crews
        ]

    if set(crew_by_element) != set(expected_elements):
        missing = sorted(set(expected_elements) - set(crew_by_element))
        additional = sorted(set(crew_by_element) - set(expected_elements))
        raise ValueError(
            f"Incomplete crew mapping. Missing={missing}; "
            f"Additional={additional}"
        )

    # Required final manuscript format:
    # Crews: List[Tuple[String, List[String]]]
    return {
        "Crews": [
            [element, crew_by_element[element]]
            for element in expected_elements
        ]
    }


def main() -> None:
    started_at = utc_now()
    start_time = time.perf_counter()

    run_record: dict[str, Any] = {
        "agent": "Ext-A-II",
        "status": "running",
        "started_at_utc": started_at,
        "completed_at_utc": None,
        "duration_seconds": None,
        "files": {
            "material_input": MATERIAL_FILE.name,
            "ext_a_i_input": EXT_A_I_FILE.name,
            "configuration": CONFIG_FILE.name,
            "output": OUTPUT_FILE.name,
            "run_record": RUN_RECORD_FILE.name,
        },
        "attempts": [],
    }

    try:
        if not os.getenv("OPENAI_API_KEY"):
            raise RuntimeError("OPENAI_API_KEY is not set.")

        material_data = read_json(MATERIAL_FILE)
        ext_a_i_result = read_json(EXT_A_I_FILE)
        config = read_json(CONFIG_FILE)

        elements = material_data["Design"]
        expected_elements = [
            element["Element"] for element in elements
        ]

        model = get_selected_model(config)
        api_config = config.get("api", {})
        agent_config = config.get("agent", {})
        max_attempts = agent_config.get(
            "validation_max_attempts",
            3,
        )

        user_prompt = f"""
Element Definitions:
{json.dumps(elements, indent=2, ensure_ascii=False)}

Ext-A-I Output:
{json.dumps(ext_a_i_result, indent=2, ensure_ascii=False)}

Required element names:
{json.dumps(expected_elements, indent=2, ensure_ascii=False)}
""".strip()

        request_parameters = build_request(config, user_prompt)

        run_record.update(
            {
                "input_hashes_sha256": {
                    MATERIAL_FILE.name: sha256_file(MATERIAL_FILE),
                    EXT_A_I_FILE.name: sha256_file(EXT_A_I_FILE),
                    CONFIG_FILE.name: sha256_file(CONFIG_FILE),
                },
                "ext_a_i_input": ext_a_i_result,
                "element_count": len(elements),
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
                    "timeout_seconds": api_config.get(
                        "timeout_seconds",
                        120,
                    ),
                    "sdk_max_retries": api_config.get(
                        "sdk_max_retries",
                        2,
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
                    "validation_max_attempts": max_attempts,
                    "parallel_tool_calls": False,
                    "tool_choice": TOOL_NAME,
                    "strict_tool_schema": True,
                    "store": config.get(
                        "request",
                        {},
                    ).get("store", False),
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
                    "user_prompt": user_prompt,
                    "user_prompt_sha256": sha256_text(
                        user_prompt
                    ),
                },
                "tool_schema": EXT_A_II_TOOL,
                "request_parameters_sent": {
                    key: value
                    for key, value in request_parameters.items()
                    if key not in {
                        "instructions",
                        "input",
                        "tools",
                    }
                },
            }
        )

        client = create_client(config)
        last_error = "Unknown validation failure."

        for attempt_number in range(1, max_attempts + 1):
            attempt_started_at = utc_now()
            attempt_start = time.perf_counter()

            attempt_record: dict[str, Any] = {
                "attempt": attempt_number,
                "started_at_utc": attempt_started_at,
                "status": "running",
            }

            response = None

            try:
                response = client.responses.create(
                    **request_parameters
                )

                tool_arguments = extract_tool_arguments(response)
                final_output = validate_and_format_crews(
                    tool_arguments,
                    expected_elements,
                )

                attempt_record.update(
                    {
                        "status": "accepted",
                        "completed_at_utc": utc_now(),
                        "duration_seconds": round(
                            time.perf_counter()
                            - attempt_start,
                            6,
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
                        "usage": usage_to_dict(response),
                        "tool_arguments": tool_arguments,
                        "formatted_output": final_output,
                    }
                )
                run_record["attempts"].append(attempt_record)

                write_json(OUTPUT_FILE, final_output)

                run_record.update(
                    {
                        "status": "success",
                        "completed_at_utc": utc_now(),
                        "duration_seconds": round(
                            time.perf_counter() - start_time,
                            6,
                        ),
                        "final_output": final_output,
                    }
                )
                write_json(RUN_RECORD_FILE, run_record)

                print(
                    json.dumps(
                        final_output,
                        indent=2,
                        ensure_ascii=False,
                    )
                )
                print(f"\nSaved: {OUTPUT_FILE}")
                print(f"Saved: {RUN_RECORD_FILE}")
                return

            except Exception as exc:
                last_error = f"{type(exc).__name__}: {exc}"

                attempt_record.update(
                    {
                        "status": "rejected",
                        "completed_at_utc": utc_now(),
                        "duration_seconds": round(
                            time.perf_counter()
                            - attempt_start,
                            6,
                        ),
                        "error": last_error,
                    }
                )

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

                run_record["attempts"].append(attempt_record)

        raise RuntimeError(
            "Ext-A-II failed after "
            f"{max_attempts} validation attempts. "
            f"Last error: {last_error}"
        )

    except Exception as exc:
        run_record.update(
            {
                "status": "failed",
                "completed_at_utc": utc_now(),
                "duration_seconds": round(
                    time.perf_counter() - start_time,
                    6,
                ),
                "failure": {
                    "type": type(exc).__name__,
                    "message": str(exc),
                },
            }
        )
        write_json(RUN_RECORD_FILE, run_record)
        raise


if __name__ == "__main__":
    main()
