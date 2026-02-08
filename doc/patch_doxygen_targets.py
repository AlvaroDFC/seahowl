"""Patch target attributes in Doxygen HTML/SVG files for iframe compatibility.

Replaces target="_top" with target="_parent" so that links in SVG diagrams
navigate within the iframe instead of breaking out to the top-level window.
"""

import pathlib
import re
import sys

doxygen_dir = pathlib.Path(sys.argv[1])
pattern = re.compile(r'target="(?:_top|top)"')

for ext in ("*.html", "*.svg"):
    for filepath in doxygen_dir.rglob(ext):
        text = filepath.read_text(encoding="utf-8")
        new_text = pattern.sub('target="_parent"', text)
        if new_text != text:
            filepath.write_text(new_text, encoding="utf-8")
