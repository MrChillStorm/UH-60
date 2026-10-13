#!/bin/sh
# Rebuild the report from the cached results (report/results/*.json; rerun report/mkresults.py to refresh them)
# and copy the PDF to the repository root.
set -e
cd "$(dirname "$0")"
python3 mktables.py
python3 figures.py
tectonic -X compile uh60a_report.tex
cp uh60a_report.pdf ../../UH60A-JSBSim-model-report.pdf
