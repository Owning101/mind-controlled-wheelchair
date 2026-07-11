import tempfile
import unittest
from pathlib import Path

import obsidian_anki_sync as sync


class ObsidianAnkiSyncTests(unittest.TestCase):
    def test_parse_arrow_and_parentheses_entries(self):
        with tempfile.TemporaryDirectory() as tmp:
            note = Path(tmp) / "vocab.md"
            note.write_text(
                "gehen -> to go\n\nBlamieren (embarrass)\n# heading\n",
                encoding="utf-8",
            )

            entries = sync.parse_vocab_note(note)

        self.assertEqual([entry.german for entry in entries], ["gehen", "Blamieren"])
        self.assertEqual([entry.user_translation for entry in entries], ["to go", "embarrass"])

    def test_build_back_includes_user_ai_and_examples(self):
        entry = sync.VocabEntry("gehen", "to go", 1)
        back = sync.build_back(
            entry,
            {
                "ai_translation": "to go; to walk",
                "translation_note": "The meaning depends on context.",
                "examples": ["Ich gehe nach Hause. - I am going home."],
            },
        )

        self.assertIn("Your translation", back)
        self.assertIn("to go", back)
        self.assertIn("to go; to walk", back)
        self.assertIn("Ich gehe nach Hause.", back)


if __name__ == "__main__":
    unittest.main()
