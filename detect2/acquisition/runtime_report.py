"""Small offline review of a combined runtime recording and its raw audit."""
from html import escape
import json
from pathlib import Path


def report(folder):
    folder=Path(folder)
    from .audit_session import audit
    raw=audit(folder)
    state=json.loads((folder/'runtime_manifest.json').read_text())
    clock_link=''
    if state.get('timesync_active') and (folder/'clock_sync.jsonl').exists():
        from .clock_report import report as clock_report
        clock=clock_report(folder)
        clock_link=f'<p><a href="timesync/report.html">Pi/PX4 TIMESYNC plots and clock mapping</a>: {clock["accepted_replies"]} accepted replies. GNSS epoch alignment remains UNKNOWN.</p>'
    files=''.join('<li><a href="'+escape(p.name)+'">'+escape(p.name)+'</a></li>' for p in
                  sorted(folder.iterdir()) if p.is_file() and p.name not in ['recording_report.html'] and not p.name.endswith(('.tmp','.ftp.part')))
    collected=state.get('native_collection')=='collected'
    body=f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>detect2 recording</title><style>body{{font:15px/1.65 system-ui;background:#eff5f7;color:#243b4c;max-width:1050px;margin:35px auto;padding:0 22px}}section{{background:white;border:1px solid #dae6ec;padding:24px;border-radius:12px;margin:20px 0}}h1{{font-size:35px}}a{{color:#176f94}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}.note{{border-left:3px solid #bc862d;padding:12px;background:#fff6e7}}</style></head><body><h1>Combined runtime recording</h1><p><b>{escape(state['status'].upper())}</b> · {escape(folder.name)}</p>
<section><h2>Acquisition</h2><p>Required streams reached ready: {state.get('recording_ready',False)}. RAWX epochs: {raw['receiver']['epochs']}. Native ULog collected: {collected}.</p><p>Receiver profile: 20 Hz / 2 Mbaud, RAM only. Native logger: 128 KiB runtime buffer. Startup files changed: {state['startup_files_changed']}. The raw recorder has stopped; native logging stops on normal disarmed finalization. Armed stop defers native collection.</p><p><a href="report.html">Raw archive integrity, GNSS intervals and telemetry audit</a> · <a href="runtime_manifest.json">Combined session status and file hashes</a></p><p class="note">Clock alignment remains UNKNOWN. This launcher does not classify spoofing. Native sample rates are checked by the separate native ULog audit; 189.6 Hz was accepted in the earlier bench test. That prior rate is not a measurement of this session.</p></section>
<section><h2>Native log and interruption handling</h2><p>Bookmarked SD-card file: <code>{escape(state.get('native_path','not started'))}</code>.</p><p>Native collection: {escape(state.get('native_collection','UNKNOWN'))}. {escape(state.get('native_collection_error',''))}</p><pre>{escape(json.dumps(state.get('errors',[]),indent=2))}</pre><p>A user stop is recorded as stopped, not a timed completion. The generic raw audit accepts only complete for its capture-session check; verify its separate archive/finalization checks for a deliberate Ctrl+C. Disconnects, clock resets and queue losses mark failure. A newer native log is never stopped to collect an older bookmarked file.</p><p>To finish a pending native collection after disarm:</p><pre>./start_recording --finalize {escape(str(folder.resolve()))}</pre></section>
<section><h2>Clock bridge</h2>{clock_link or '<p>Active TIMESYNC disabled for this recording.</p>'}</section>
<section><h2>Files</h2><ul>{files}</ul></section></body></html>'''
    (folder/'recording_report.html').write_text(body)
    print('Combined report:',folder/'recording_report.html',flush=True)
