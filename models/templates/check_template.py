"""Check the pinned router template using llama.cpp's real chat renderer."""

import configparser
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: check_template.py /path/to/test-chat-template")
    renderer = Path(sys.argv[1]).resolve(strict=True)
    directory = Path(__file__).resolve().parent
    template = directory / "mistral-v7-tekken.jinja"
    expected_hash = "338c134a61cf54e1d250a8f6c2e974c788c14ae97dd6e7b2491d5446ec472490"
    assert hashlib.sha256(template.read_bytes()).hexdigest() == expected_hash
    config = configparser.ConfigParser(interpolation=None)
    config.read(directory.parent / "llama-router.ini")
    preset = config["ministral-3-8b-instruct-2512-abliterated"]
    assert preset["jinja"] == "on"
    assert "chat-template" not in preset
    assert preset["chat-template-file"] == f"/home/oppa-ai/AGi/models/templates/{template.name}"

    system = {"role": "system", "content": "SYSTEM_SENTINEL: You are AIVA."}
    user = {"role": "user", "content": "USER_SENTINEL"}
    tool = {"type": "function", "function": {
        "name": "weather_probe", "description": "TOOL_SCHEMA_SENTINEL",
        "parameters": {"type": "object", "properties": {
            "city": {"type": "string"}}, "required": ["city"]}}}
    call = {"role": "assistant", "content": "", "tool_calls": [{
        "id": "call00001", "type": "function", "function": {
            "name": "weather_probe", "arguments": '{"city":"CITY_SENTINEL"}'}}]}
    cases = [
        ("system/user", [system, user], [], [
            "[SYSTEM_PROMPT]SYSTEM_SENTINEL: You are AIVA.[/SYSTEM_PROMPT]",
            "[INST]USER_SENTINEL[/INST]"]),
        ("multi-turn", [system, user,
            {"role": "assistant", "content": "ASSISTANT_SENTINEL"},
            {"role": "user", "content": "FOLLOWUP_SENTINEL"}], [], [
                "USER_SENTINEL", "ASSISTANT_SENTINEL</s>",
                "[INST]FOLLOWUP_SENTINEL[/INST]"]),
        ("tool round trip", [system, user, call,
            {"role": "tool", "tool_call_id": "call00001",
             "content": "TOOL_RESULT_SENTINEL"}], [tool], [
                "[AVAILABLE_TOOLS]", "TOOL_SCHEMA_SENTINEL",
                "[TOOL_CALLS]weather_probe[ARGS]", "CITY_SENTINEL",
                "[TOOL_RESULTS]TOOL_RESULT_SENTINEL[/TOOL_RESULTS]"]),
    ]
    with tempfile.TemporaryDirectory() as temp:
        fixture = Path(temp) / "input.json"
        output = Path(temp) / "output.txt"
        for name, messages, tools, expected in cases:
            fixture.write_text(json.dumps({"messages": messages, "tools": tools}))
            output.unlink(missing_ok=True)
            result = subprocess.run([str(renderer), "--json", str(fixture),
                "--output", str(output), str(template)], capture_output=True,
                text=True, timeout=60)
            assert result.returncode == 0, result.stdout + result.stderr
            rendered = output.read_text()
            assert all(part in rendered for part in expected), (name, rendered)
            assert "You power an AI assistant called Le Chat" not in rendered
            print(f"PASS: {name}")
        fixture.write_text(json.dumps({"messages": [], "tools": []}))
        result = subprocess.run([str(renderer), "--json", str(fixture), str(template)],
            capture_output=True, text=True, timeout=60)
        assert result.returncode != 0, "Empty conversation must be rejected"
        assert "Conversation must have at least one message" in result.stdout + result.stderr
        print("PASS: empty conversation rejected")
    print("PASS: pinned source and router configuration")


if __name__ == "__main__":
    main()
