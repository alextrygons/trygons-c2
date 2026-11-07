"""The talking manual (CLAUDE.md section W, docs/assistant_design.md): an offline voice assistant on the
STATION PC that answers questions about the C2 from the manual. READ-ONLY - it never sends anything to the
boat or the gateway. client.py = the station side + the only process door; worker.py = its own process.
"""
