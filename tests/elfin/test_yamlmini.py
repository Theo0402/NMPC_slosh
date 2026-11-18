"""The stdlib YAML subset reads every config file as PyYAML does."""
import pathlib

import pytest
import yaml

from tools import yamlmini

ROOT = pathlib.Path(__file__).resolve().parents[2]
STDLIB_CONFIGS = ["config/arm/confirmed.yaml", "config/arm/e03pro.yaml", "config/cup/r40.yaml",
                  "config/camera.yaml", "config/arm/fk_poses.yaml",
                  "config/workbench/stages.yaml", "config/params.schema.yaml"]   # the last two: the workbench


@pytest.mark.parametrize("rel", STDLIB_CONFIGS)
def test_config_files_agree_with_pyyaml(rel):
    text = (ROOT / rel).read_text(encoding="utf-8")
    assert yamlmini.loads(text, rel) == yaml.safe_load(text)


def test_params_schema_is_valid_yaml():
    # read by the generators (PyYAML), not by arm tools
    d = yaml.safe_load((ROOT / "config/params.schema.yaml").read_text(encoding="utf-8"))
    assert d["fields"]["omega0"]["unit"] == "rad/s"


@pytest.mark.parametrize("text", [
    "a: yes", "a: 1e-3", "a: .5", "a: 0x10", "a: 1_000", "a: {b: 1}", "a: &x 1", "a: *x",
    "a:\n\tb: 1", "a: 1\na: 2", "- a: 1", "a: b: c", "a: |\n  x", "---\na: 1",
])
def test_unsupported_or_ambiguous_yaml_is_refused(text):
    with pytest.raises(yamlmini.YamlError):
        yamlmini.loads(text)


def test_subset_values():
    text = "a: 1\nb: -2.5\nc: [1.0, [2, 3], 'x y']\nd:\n  e: null\n  f: true\ng:\n  - 1.0\n  - [2, 3]\nh: 192.168.0.10  # ip\n"
    assert yamlmini.loads(text) == yaml.safe_load(text)
