"""Generator rules + drift gate for scripts/gen_protocol.py.

- `--check` runs as part of the normal test entry, so committed generated
  outputs that drift from protocol.yaml FAIL the tests.
- Every validation rule has a negative test: a bad schema must fail generation
  with a SchemaError, never emit silently-wrong code.
"""
import copy
import os
import subprocess
import sys
import tempfile
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import gen_protocol as G  # noqa: E402

# Minimal valid schema the negative tests mutate.
BASE = {
    "meta": {"proto_version": 1, "endianness": "little"},
    "constants": [{"name": "N", "value": 2}],
    "enums": [{"name": "E", "wire_type": "u8",
               "values": [{"name": "E_A", "value": 0},
                          {"name": "E_B", "value": 1}]}],
    "flags": [{"name": "f", "bits": [{"bit": 0, "name": "F_X"},
                                     {"bit": 1, "name": "F_Y"}]}],
    "scales": [{"name": "S", "value": 1.0}],
    "structs": [
        {"name": "inner_t", "fields": [{"name": "a", "type": "u8"}]},
        {"name": "outer_t", "fields": [
            {"name": "n", "type": "u8"},
            {"name": "arr", "type": "inner_t", "count": "N",
             "truncate_by": "n"},
        ]},
    ],
    "derived": [{"name": "D", "expr": "sizeof(outer_t) + 1"}],
}


def _load(schema_dict):
    """Round-trip the dict through a YAML file into load_schema."""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump(schema_dict, f)
        path = f.name
    try:
        return G.load_schema(path)
    finally:
        os.unlink(path)


class DriftGate(unittest.TestCase):
    def test_check_passes_on_committed_outputs(self):
        r = subprocess.run([sys.executable,
                            os.path.join(ROOT, "scripts", "gen_protocol.py"),
                            "--check"], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("up to date", r.stdout)

    def test_check_flags_stale_output(self):
        schema = G.load_schema()
        layout = G.compute_layout(schema)
        derived = G.compute_derived(schema, layout)
        shash = G.schema_hash(schema)
        with tempfile.NamedTemporaryFile("w", suffix=".h", delete=False) as f:
            f.write("/* stale */\n")
            stale_path = f.name
        orig = G.EMITTERS["c"]
        try:
            G.EMITTERS["c"] = (orig[0], stale_path)
            stale = G.check_outputs(schema, layout, derived, shash, ["c"])
        finally:
            G.EMITTERS["c"] = orig
            os.unlink(stale_path)
        self.assertEqual(len(stale), 1)


class SchemaRules(unittest.TestCase):
    def _expect_fail(self, schema_dict, fragment):
        with self.assertRaises(G.SchemaError) as cm:
            _load(schema_dict)
        self.assertIn(fragment, str(cm.exception))

    def test_base_schema_is_valid(self):
        s = _load(BASE)
        layout = G.compute_layout(s)
        self.assertEqual(layout["outer_t"]["size"], 3)
        self.assertEqual(G.compute_derived(s, layout)["D"], 4)

    def test_field_after_truncated_array(self):
        s = copy.deepcopy(BASE)
        s["structs"][1]["fields"].append({"name": "tail", "type": "u8"})
        self._expect_fail(s, "follows the truncate_by array")

    def test_unknown_field_type(self):
        s = copy.deepcopy(BASE)
        s["structs"][0]["fields"][0]["type"] = "u64"
        self._expect_fail(s, "unknown type")

    def test_forward_struct_reference(self):
        # A struct may only embed PREVIOUSLY defined structs.
        s = copy.deepcopy(BASE)
        s["structs"][0]["fields"].append(
            {"name": "o", "type": "outer_t"})
        self._expect_fail(s, "unknown type")

    def test_duplicate_names(self):
        s = copy.deepcopy(BASE)
        s["constants"].append({"name": "E_A", "value": 5})
        self._expect_fail(s, "duplicate name")

    def test_duplicate_field_names(self):
        s = copy.deepcopy(BASE)
        s["structs"][0]["fields"].append({"name": "a", "type": "u8"})
        self._expect_fail(s, "duplicate field")

    def test_enum_value_overflows_wire_type(self):
        s = copy.deepcopy(BASE)
        s["enums"][0]["values"].append({"name": "E_BIG", "value": 256})
        self._expect_fail(s, "does not fit u8")

    def test_duplicate_enum_value(self):
        s = copy.deepcopy(BASE)
        s["enums"][0]["values"].append({"name": "E_DUP", "value": 0})
        self._expect_fail(s, "duplicate value")

    def test_overlapping_flag_bits(self):
        s = copy.deepcopy(BASE)
        s["flags"][0]["bits"].append({"bit": 0, "name": "F_Z"})
        self._expect_fail(s, "overlapping bit")

    def test_yaml_bool_coercion_rejected(self):
        # PyYAML 1.1 turns an unquoted `no`/`on` into a bool; the generator
        # must reject it instead of emitting it as 0/1.
        s = copy.deepcopy(BASE)
        s["constants"][0]["value"] = False
        self._expect_fail(s, "not an integer")

    def test_count_must_be_known_constant(self):
        s = copy.deepcopy(BASE)
        s["structs"][1]["fields"][1]["count"] = "MISSING"
        self._expect_fail(s, "not a defined constant")

    def test_truncate_by_must_be_earlier_field(self):
        s = copy.deepcopy(BASE)
        s["structs"][1]["fields"][1]["truncate_by"] = "nope"
        self._expect_fail(s, "not an earlier field")

    def test_usb_budget_overflow_fails_generation(self):
        s = copy.deepcopy(BASE)
        s["constants"].append({"name": "USB_TX_SLOT_MAX", "value": 2})
        s["derived"].append({"name": "USB_MAX_FRAME_SIZE",
                             "expr": "sizeof(outer_t)"})
        parsed = _load(s)
        layout = G.compute_layout(parsed)
        with self.assertRaises(G.SchemaError) as cm:
            G.compute_derived(parsed, layout)
        self.assertIn("exceeds USB_TX_SLOT_MAX", str(cm.exception))

    def test_derived_eval_rejects_junk(self):
        s = _load(BASE)
        layout = G.compute_layout(s)
        for expr in ("sizeof(nope)", "N +", "N ** 2", "__import__", "1 - 1"):
            with self.assertRaises(G.SchemaError):
                G.eval_derived(expr, {"N": 2}, layout, {})


if __name__ == "__main__":
    unittest.main()
