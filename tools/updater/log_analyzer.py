"""
log_analyzer.py - Parse rotating log files and plot one or more chosen metrics.

Prerequisites (install once):
    pip install plotly
    
Usage:
    python log_analyzer.py  # Uses default phrase and folder
    python log_analyzer.py --phrase "PWM pulse:" --folder other_logs
    python log_analyzer.py --phrase "PWM pulse:" --phrase "Encoder position:"
"""

import argparse
import glob
import os
import re
from datetime import datetime
import plotly.graph_objs as go
from collections import defaultdict

# === Configuration defaults === #
DEFAULT_FOLDER = "logs"
DEFAULT_PREFIX = "system.log"
DEFAULT_PHRASES = ["Encoder position:"]  # Text to search for
# === #

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Plot log values over time.")
    ap.add_argument("--folder", default=DEFAULT_FOLDER,
                    help="Folder containing log files")
    ap.add_argument("--prefix", default=DEFAULT_PREFIX,
                    help="File prefix, e.g. system.log (rotated logs .1, .2 ... will be included)")
    ap.add_argument("--phrase", action="append", default=None,
                    help="Phrase(s) that precede numeric values (can be used multiple times)")
    return ap.parse_args()

def parse_timestamp(ts_string):
    """
    Try parsing with and without microsecond precision.
    """
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(ts_string, fmt)
        except ValueError:
            continue
    raise ValueError(f"Unrecognized timestamp format: {ts_string}")

def collect_values(folder: str, prefix: str, phrases: list[str]):
    """
    Return a dict: {phrase: list of (timestamp, value)}
    """
    pattern = os.path.join(folder, prefix + "*")
    files = sorted(glob.glob(pattern))
    
    phrase_data = defaultdict(list)
    
    # Build regexes per phrase
    regexes = {
        phrase: re.compile(
            rf"^(?P<ts>\d{{4}}-\d{{2}}-\d{{2}} \d{{2}}:\d{{2}}:\d{{2}}(?:\.\d{{1,6}})?)"
            rf".*?{re.escape(phrase)}\s*(?P<val>-?\d+\.?\d*)"
        )
        for phrase in phrases
    }
    
    for fname in files:
        with open(fname, "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                for phrase, regex in regexes.items():
                    m = regex.search(line)
                    if m:
                        ts = parse_timestamp(m.group("ts"))
                        val = float(m.group("val"))
                        phrase_data[phrase].append((ts, val))
    return phrase_data

def main():
    args = parse_args()
    
    # If no --phrase was passed, use the default
    phrases = args.phrase if args.phrase else ["Encoder position:"]
    
    data_by_phrase = collect_values(args.folder, args.prefix, phrases)
    if not data_by_phrase:
        print(f"No matches found for phrases in {args.folder}/{args.prefix}*")
        return
    
    fig = go.Figure()
    
    for phrase, data in data_by_phrase.items():
        if not data:
            continue
        timestamps, values = zip(*data)
        fig.add_trace(go.Scatter(
            x=timestamps,
            y=values,
            mode='lines+markers',
            name=phrase,
            hovertemplate="%{y}<extra></extra>"
        ))
    
    fig.update_layout(
        title="Log values over time",
        xaxis_title="Time",
        yaxis_title="Value",
        hovermode="x unified"
    )
    
    fig.show()
    
if __name__ == "__main__":
    main()