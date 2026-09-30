# SPDX-FileCopyrightText: 2026 Olivier Guyon et al
#
# SPDX-License-Identifier: LGPL-3.0-or-later

"""Sphinx configuration for the cacao documentation.

Build with ``tox -e docs`` from the repository root.
"""

project = "cacao"
author = "Olivier Guyon et al"
copyright = "2026, Olivier Guyon et al"

extensions = [
    "myst_parser",
]

source_suffix = {
    ".md": "markdown",
}

myst_enable_extensions = [
    "colon_fence",
    "deflist",
]
# Generate anchors for headings so cross-page `file.md#heading` links resolve.
myst_heading_anchors = 3

exclude_patterns = ["_build"]

html_theme = "furo"
html_title = "cacao"
html_logo = "../cacao-logo-250pix.png"
