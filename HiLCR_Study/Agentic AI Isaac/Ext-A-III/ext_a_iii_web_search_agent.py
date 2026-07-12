#!/usr/bin/env python3
"""
Ext-A-III using OpenAI o3 web search instead of an O*NET API key.

Expected files in the same directory as this script:
    Ext_A_I_Output.json
    Ext_A_II_Output.json
    config.json

Required environment variable:
    OPENAI_API_KEY

Created files:
    Ext_A_III_Output.json
    Ext_A_III_Run_Record.json
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
from urllib.parse import urlparse

from openai import OpenAI


BASE_DIR = Path(__file__).resolve().parent

EXT_A_I_FILE = BASE_DIR / "Ext_A_I_Output.json"
EXT_A_II_FILE = BASE_DIR / "Ext_A_II_Output.json"
CONFIG_FILE = BASE_DIR / "config.json"

OUTPUT_FILE = BASE_DIR / "Ext_A_III_Output.json"
RUN_RECORD_FILE = BASE_DIR / "Ext_A_III_Run_Record.json"

TOOL_NAME = "submit_ext_a_iii_tasks"

DEFAULT_ONET_DOMAINS = [
    "onetonline.org",
    "onetcenter.org",
]


EXT_A_III_TOOL: dict[str, Any] = {
    "type": "function",
    "name": TOOL_NAME,
    "description": (
        "Submit O*NET task descriptions associated with each supplied "
        "human crew type."
    ),
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "Tasks": {
                "type": "array",
                "description": "One task mapping for every supplied crew type.",
                "items": {
                    "type": "object",
                    "properties": {
                        "CrewType": {
                            "type": "string",
                            "description": (
                                "The exact crew type copied from Ext-A-II."
                            ),
                        },
                        "TaskDescriptions": {
                            "type": "array",
                            "description": (
                                "Relevant O*NET task descriptions. Return "
                                "a single null value when no valid task was found."
                            ),
                            "items": {
                                "anyOf": [
                                    {"type": "string"},
                                    {"type": "null"},
                                ]
                            },
                        },
                    },
                    "required": ["CrewType", "TaskDescriptions"],
                    "additionalProperties": False,
                },
            }
        },
        "required": ["Tasks"],
        "additionalProperties": False,
    },
}


SEARCH_INSTRUCTIONS = """
You are the web-retrieval component of Ext-A-III.

Search only official O*NET websites for the crew type supplied by the user.
Identify the closest relevant O*NET occupation and retrieve task statements
that may apply during construction assembly installation.

Requirements:
- Use O*NET OnLine or the O*NET Resource Center only.
- Report the matched occupation title and O*NET-SOC code.
- Copy relevant O*NET task statements as written on the source page.
- Do not invent or paraphrase task statements.
- Prefer occupation-level Tasks over general work activities.
- Include the source page URLs in the response.
- If no defensible occupation or task is found, state that clearly.
""".strip()


FORMAT_INSTRUCTIONS = """
You are Ext-A-III in a construction knowledge-extraction pipeline.

Use the supplied Ext-A-I context, Ext-A-II crew types, and the web-search
evidence retrieved from official O*NET websites.

Requirements:
- Return exactly one mapping for every supplied crew type.
- Copy every crew type exactly as supplied.
- Select only task statements supported by the retrieved O*NET evidence.
- Keep the O*NET task wording unchanged.
- Select tasks relevant to construction or assembly installation.
- If a specific crew has no valid supported task, return [null] for that crew.
- At least one crew must have at least one valid task description.
- Return the result only through the submit_ext_a_iii_tasks tool.
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


def response_sources(response: Any) -> list[dict[str, Any]]:
    """
    Extract the complete source list returned by the hosted web-search tool.
    """
    if not hasattr(response, "model_dump"):
        return []

    response_data = response.model_dump(mode="json")
    sources: list[dict[str, Any]] = []

    for item in response_data.get("output", []):
        if item.get("type") != "web_search_call":
            continue

        action = item.get("action") or {}
        for source in action.get("sources") or []:
            if source not in sources:
                sources.append(source)

    return sources


def get_selected_model(config: dict[str, Any]) -> dict[str, Any]:
    profile_name = config["active_model_profile"]
    return config["model_profiles"][profile_name]


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


def model_parameters(config: dict[str, Any]) -> dict[str, Any]:
    model = get_selected_model(config)

    parameters: dict[str, Any] = {
        "model": model["identifier"],
        "max_output_tokens": model.get("max_output_tokens", 4096),
        "store": config.get("request", {}).get("store", False),
    }

    if model.get("reasoning_effort") is not None:
        parameters["reasoning"] = {
            "effort": model["reasoning_effort"]
        }

    if model.get("temperature") is not None:
        parameters["temperature"] = model["temperature"]

    if model.get("top_p") is not None:
        parameters["top_p"] = model["top_p"]

    return parameters


def extract_crew_types(ext_a_ii: dict[str, Any]) -> list[str]:
    """
    Expected Ext-A-II format:
        {
          "Crews": [
            ["Vertical Stud", ["Carpenter"]],
            ["OSB Sheathing Plate", ["Carpenter", "Laborer"]]
          ]
        }
    """
    if "Ext-A-II" in ext_a_ii:
        ext_a_ii = ext_a_ii["Ext-A-II"]

    unique_crews: list[str] = []

    for element_and_crews in ext_a_ii["Crews"]:
        for crew in element_and_crews[1]:
            if crew not in unique_crews:
                unique_crews.append(crew)

    return unique_crews


def used_web_search(response: Any) -> bool:
    return any(
        getattr(item, "type", None) == "web_search_call"
        for item in response.output
    )


def official_onet_sources(response: Any) -> list[dict[str, Any]]:
    official_sources: list[dict[str, Any]] = []

    for source in response_sources(response):
        url = source.get("url", "")
        hostname = (urlparse(url).hostname or "").lower()

        if any(
            hostname == domain or hostname.endswith(f".{domain}")
            for domain in DEFAULT_ONET_DOMAINS
        ):
            official_sources.append(source)

    return official_sources


def search_onet_for_crew(
    client: OpenAI,
    config: dict[str, Any],
    crew: str,
    ext_a_i: dict[str, Any],
) -> tuple[Any, str]:
    web_config = config.get("web_search", {})
    allowed_domains = web_config.get(
        "allowed_domains",
        DEFAULT_ONET_DOMAINS,
    )
    max_tasks = web_config.get("max_tasks_per_crew", 10)
    max_search_attempts = web_config.get("max_search_attempts", 3)

    # Azure/LiteLLM currently exposes o3 web search through the legacy
    # web_search_preview tool. That tool accepts tool_choice="auto" but does
    # not accept domain filters or external_web_access.
    tool_type = web_config.get(
        "tool_type",
        "web_search_preview",
    )
    tool_choice = web_config.get("tool_choice", "auto")

    if tool_type == "web_search_preview":
        tool_choice = "auto"

    domain_query = " OR ".join(
        f"site:{domain}" for domain in allowed_domains
    )

    base_prompt = f"""
You must use the web-search tool before answering.

Crew type: {crew}

Assembly context from Ext-A-I:
{json.dumps(ext_a_i, indent=2, ensure_ascii=False)}

Search query scope:
({domain_query}) "{crew}" occupation tasks O*NET

Search only official O*NET pages for the occupation that most closely
represents this crew type. Return up to {max_tasks} task statements that are
relevant to construction or assembly installation. Preserve the task wording
exactly and identify the matched occupation title, O*NET-SOC code, and source
URLs. Do not rely on non-O*NET websites.
""".strip()

    web_tool: dict[str, Any] = {"type": tool_type}

    # These controls are supported by the newer web_search tool, but not by
    # web_search_preview used by the Azure/LiteLLM o3 route.
    if tool_type == "web_search":
        web_tool["filters"] = {
            "allowed_domains": allowed_domains,
        }
        web_tool["external_web_access"] = True

    last_response: Any | None = None
    last_prompt = base_prompt

    for attempt in range(1, max_search_attempts + 1):
        prompt = base_prompt

        if attempt > 1:
            prompt += (
                "\n\nA prior attempt did not provide an official O*NET "
                "web-search source. You must perform the search now and use "
                "at least one source from onetonline.org or onetcenter.org."
            )

        request = {
            **model_parameters(config),
            "instructions": SEARCH_INSTRUCTIONS,
            "input": prompt,
            "tools": [web_tool],
            "tool_choice": tool_choice,
            "include": ["web_search_call.action.sources"],
        }

        response = client.responses.create(**request)
        last_response = response
        last_prompt = prompt

        if used_web_search(response) and official_onet_sources(response):
            return response, prompt

    if last_response is None:
        raise RuntimeError(
            f"No web-search response was returned for crew {crew!r}."
        )

    raise RuntimeError(
        f"Web search for crew {crew!r} did not return an official O*NET "
        f"source after {max_search_attempts} attempts."
    )


def extract_function_arguments(response: Any) -> dict[str, Any]:
    for item in response.output:
        if item.type == "function_call" and item.name == TOOL_NAME:
            return json.loads(item.arguments)

    raise RuntimeError(f"The model did not call {TOOL_NAME}.")


def validate_and_format(
    arguments: dict[str, Any],
    crew_types: list[str],
) -> dict[str, Any]:
    returned: dict[str, list[str | None]] = {}

    for item in arguments.get("Tasks", []):
        crew = item.get("CrewType")
        tasks = item.get("TaskDescriptions")

        if (
            not isinstance(crew, str)
            or not isinstance(tasks, list)
            or not tasks
        ):
            raise ValueError(
                "Each crew must have a non-empty task list or [null]."
            )

        returned[crew] = tasks

    if set(returned) != set(crew_types):
        missing = sorted(set(crew_types) - set(returned))
        extra = sorted(set(returned) - set(crew_types))
        raise ValueError(
            f"Missing crews={missing}; additional crews={extra}."
        )

    if not any(
        task is not None
        for crew in crew_types
        for task in returned[crew]
    ):
        raise ValueError(
            "No valid task was identified for any crew type."
        )

    # Requested final format:
    # Tasks: List[Tuple[String, List[String]]]
    return {
        "Tasks": [
            [crew, returned[crew]]
            for crew in crew_types
        ]
    }


def main() -> None:
    started = time.perf_counter()

    run_record: dict[str, Any] = {
        "agent": "Ext-A-III",
        "retrieval_method": "OpenAI Responses API hosted web search",
        "status": "running",
        "started_at_utc": utc_now(),
        "completed_at_utc": None,
        "duration_seconds": None,
        "files": {
            "ext_a_i_input": EXT_A_I_FILE.name,
            "ext_a_ii_input": EXT_A_II_FILE.name,
            "configuration": CONFIG_FILE.name,
            "output": OUTPUT_FILE.name,
            "run_record": RUN_RECORD_FILE.name,
        },
        "web_searches": [],
        "formatting_attempts": [],
    }

    try:
        if not os.getenv("OPENAI_API_KEY"):
            raise RuntimeError("OPENAI_API_KEY is not set.")

        ext_a_i = read_json(EXT_A_I_FILE)
        ext_a_ii = read_json(EXT_A_II_FILE)
        config = read_json(CONFIG_FILE)

        crew_types = extract_crew_types(ext_a_ii)
        if not crew_types:
            raise RuntimeError(
                "No crew types were found in Ext_A_II_Output.json."
            )

        model = get_selected_model(config)
        api_config = config.get("api", {})
        web_config = config.get("web_search", {})
        max_attempts = config.get("agent", {}).get(
            "validation_max_attempts",
            3,
        )

        run_record.update(
            {
                "input_hashes_sha256": {
                    EXT_A_I_FILE.name: sha256_file(EXT_A_I_FILE),
                    EXT_A_II_FILE.name: sha256_file(EXT_A_II_FILE),
                    CONFIG_FILE.name: sha256_file(CONFIG_FILE),
                },
                "ext_a_i_input": ext_a_i,
                "crew_types": crew_types,
                "configuration": {
                    "active_model_profile": config[
                        "active_model_profile"
                    ],
                    "requested_model_identifier": model["identifier"],
                    "api_family": api_config.get("api_family"),
                    "api_version": api_config.get("api_version"),
                    "reasoning_effort": model.get("reasoning_effort"),
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
                    "web_search_allowed_domains": web_config.get(
                        "allowed_domains",
                        DEFAULT_ONET_DOMAINS,
                    ),
                    "web_search_tool_type": web_config.get(
                        "tool_type",
                        "web_search_preview",
                    ),
                    "web_search_tool_choice": (
                        "auto"
                        if web_config.get(
                            "tool_type",
                            "web_search_preview",
                        ) == "web_search_preview"
                        else web_config.get("tool_choice", "auto")
                    ),
                    "web_search_max_attempts": web_config.get(
                        "max_search_attempts",
                        3,
                    ),
                    "formatting_tool_choice": TOOL_NAME,
                    "strict_function_schema": True,
                },
                "software": {
                    "python_version": platform.python_version(),
                    "openai_python_version": (
                        importlib.metadata.version("openai")
                    ),
                },
                "prompts": {
                    "search_instructions": SEARCH_INSTRUCTIONS,
                    "search_instructions_sha256": sha256_text(
                        SEARCH_INSTRUCTIONS
                    ),
                    "format_instructions": FORMAT_INSTRUCTIONS,
                    "format_instructions_sha256": sha256_text(
                        FORMAT_INSTRUCTIONS
                    ),
                },
                "tool_schema": EXT_A_III_TOOL,
            }
        )

        client = create_client(config)

        evidence_by_crew: dict[str, Any] = {}

        # One official O*NET web search per unique crew type.
        for crew in crew_types:
            search_started = time.perf_counter()
            response, search_prompt = search_onet_for_crew(
                client,
                config,
                crew,
                ext_a_i,
            )

            evidence_by_crew[crew] = {
                "search_output": response.output_text,
                "sources": official_onet_sources(response),
            }

            run_record["web_searches"].append(
                {
                    "crew_type": crew,
                    "status": getattr(response, "status", None),
                    "response_id": getattr(response, "id", None),
                    "returned_model_identifier": getattr(
                        response,
                        "model",
                        None,
                    ),
                    "duration_seconds": round(
                        time.perf_counter() - search_started,
                        6,
                    ),
                    "usage": usage_to_dict(response),
                    "prompt": search_prompt,
                    "prompt_sha256": sha256_text(search_prompt),
                    "output_text": response.output_text,
                    "sources": official_onet_sources(response),
                }
            )

        formatting_prompt = f"""
Ext-A-I Output:
{json.dumps(ext_a_i, indent=2, ensure_ascii=False)}

Crew Types from Ext-A-II:
{json.dumps(crew_types, indent=2, ensure_ascii=False)}

Official O*NET web-search evidence:
{json.dumps(evidence_by_crew, indent=2, ensure_ascii=False)}
""".strip()

        formatting_request = {
            **model_parameters(config),
            "instructions": FORMAT_INSTRUCTIONS,
            "input": formatting_prompt,
            "tools": [EXT_A_III_TOOL],
            "tool_choice": {
                "type": "function",
                "name": TOOL_NAME,
            },
            "parallel_tool_calls": False,
        }

        last_error = "Unknown formatting failure."

        for attempt_number in range(1, max_attempts + 1):
            attempt_started = time.perf_counter()

            try:
                response = client.responses.create(
                    **formatting_request
                )
                arguments = extract_function_arguments(response)
                final_output = validate_and_format(
                    arguments,
                    crew_types,
                )

                run_record["formatting_attempts"].append(
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
                    }
                )

                write_json(OUTPUT_FILE, final_output)

                run_record.update(
                    {
                        "status": "success",
                        "completed_at_utc": utc_now(),
                        "duration_seconds": round(
                            time.perf_counter() - started,
                            6,
                        ),
                        "formatting_prompt": formatting_prompt,
                        "formatting_prompt_sha256": sha256_text(
                            formatting_prompt
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

                run_record["formatting_attempts"].append(
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

        raise RuntimeError(
            "Ext-A-III failed after "
            f"{max_attempts} formatting attempts. "
            f"Last error: {last_error}"
        )

    except Exception as exc:
        run_record.update(
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
        write_json(RUN_RECORD_FILE, run_record)
        raise


if __name__ == "__main__":
    main()
