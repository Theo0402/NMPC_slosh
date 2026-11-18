"""elfin_min.py stays unchanged from the knowledge files (codebase plan §2, R-B)."""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[2]


def test_elfin_min_is_byte_identical_to_the_knowledge_file():
    md = (ROOT / "docs/plans/elfin_examples.md").read_text(encoding="utf-8")
    block = re.findall(r"```python\n(.*?)```", md, re.S)[0]
    assert block.startswith("# elfin_min.py: minimal client")
    assert (ROOT / "tools/elfin_py/elfin_min.py").read_bytes() == block.encode("utf-8")


def test_no_force_sensor_commands_anywhere_in_the_tools():
    # owner, week 1: no force-sensor code is written or run
    banned = ("SetForceZero", "ReadFTCabData", "SetFTPosition", "SetForceControlState",
              "ReadFTControlState", "HRSetForceDataLimit")
    for p in (ROOT / "tools").rglob("*.py"):
        if p.name == "elfin_min.py":
            continue
        text = p.read_text(encoding="utf-8")
        assert not any(b in text for b in banned), p
