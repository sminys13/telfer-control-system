const assert = require('node:assert/strict');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');

(async () => {
  const browser = await chromium.launch({ channel: 'msedge', headless: true });
  try {
    const page = await browser.newPage();
    const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.goto(pathToFileURL(path.resolve('tools/telfer_service_console/index.html')).href);
    const results = await page.evaluate(() => {
      const checks = {};
      const info = '@SERVICE_INFO fw=step9j build=READONLY audit=1 diagnosticLock=1 auto=0';
      const start = scan => processLine(`@AUDIT state=START scan=${scan} rows=93 drives=4 fc=03 writes=0 motionPermit=0`);
      const row = (scan, drive, addr, key='P0.02', reg='F002', value=2) =>
        processLine(`@AUDIT_ROW scan=${scan} drive=${drive} addr=${addr} key=${key} reg=0x${reg} ok=1 value=${value} error=0 exception=0 ms=100`);
      const reset = () => { resetDiagnosticSession(); port={}; processLine(info); };
      reset(); start(1); row(1,0,1); processLine(info);
      checks.info_preserves_scan = audit.active && audit.received===1;
      reset(); start(2); row(2,0,1); processLine('@AUDIT state=CANCELLED scan=1');
      checks.stale_cancel_ignored = audit.active && audit.received===1;
      reset(); processLine('@AUDIT state=START scan=3 rows=1 drives=1 fc=06 writes=1 motionPermit=1');
      checks.invalid_start_rejected = !audit.active;
      reset(); start(4); row(4,0,1); row(4,0,9,'P0.03','F003',9);
      checks.address_change_rejected = audit.received===1 && !audit.rows.has('P0.03');
      reset(); start(5); row(5,0,1); row(5,1,1);
      checks.duplicate_address_rejected = audit.received===1;
      reset(); start(6); row(6,0,1); row(6,0,1,'P0.02','F002',1);
      checks.conflicting_row_rejected = audit.rows.get('P0.02')[0].value===2;
      for (const meta of HE200_AUDIT_MAP) for (let i=0;i<4;i++) {
        if (meta.name==='P0.02' && i===0) continue;
        row(6,i,i+1,meta.name,meta.reg.toString(16),meta.expected ?? 0);
      }
      processLine('@AUDIT state=DONE scan=6 errors=0');
      checks.conflict_prevents_complete = !audit.complete;
      reset(); start(7);
      for (const meta of HE200_AUDIT_MAP) for (let i=0;i<4;i++)
        row(7,i,i+1,meta.name,meta.reg.toString(16),meta.expected ?? 0);
      processLine('@AUDIT state=DONE scan=7 errors=1');
      checks.error_count_mismatch_rejected = !audit.complete;
      reset(); start(8); row(8,0,1); processLine('FW: v6-system-step9j-he200-readonly-audit');
      checks.boot_clears_snapshot = !audit.active && audit.received===0 && !auditSupported;
      reset(); processLine('@SERVICE_INFO fw=other build=FIELD audit=1 diagnosticLock=0 auto=1');
      checks.unlocked_firmware_cannot_enable_stop = $('stopAllBtn').disabled;
      checks.assist_stop_disabled = $('assistStopBtn').disabled;
      return checks;
    });
    for (const [name, ok] of Object.entries(results)) console.log(`${ok ? 'PASS' : 'FAIL'}: ${name}`);
    assert.deepEqual(errors, []);
    assert(Object.values(results).every(Boolean), 'audit integrity regressions');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode=1; });
