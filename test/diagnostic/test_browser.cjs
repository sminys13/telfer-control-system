const assert=require('node:assert/strict');
const path=require('node:path');
const {pathToFileURL}=require('node:url');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1100}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto(pathToFileURL(path.resolve('tools/telfer_service_console/index.html')).href);
  await page.waitForLoadState('load');assert.deepEqual(errors,[]);
  assert.equal(await page.locator('#auditTable tbody tr').count(),93);
  assert.equal(await page.locator('#auditAllBtn').isDisabled(),true);
  // Real DOM bubbling regression: a button click must invoke exactly one handler.
  const calls=await page.evaluate(()=>{
    const calls=[];sendPulse=(d,dir)=>calls.push([d,dir]);
    const button=document.querySelector('button[data-pulse-drive="H1"][data-dir="pos"]');
    button.disabled=false;button.click();return calls;
  });assert.deepEqual(calls,[['H1','pos']]);
  await page.reload();
  const result=await page.evaluate(async()=>{
    const commands=[];port={};writer={write:async b=>commands.push(new TextDecoder().decode(b))};
    processLine('@SERVICE_INFO fw=step9i build=FIELD native=1 auto=0');
    const oldBlocked=$('auditAllBtn').disabled;
    await send('service pulse h1 undefined 10 1200');await send('service pulse h1 pos 10 1200');
    await send('he200 probe all');await send('clear');await send('he200 audit all\nservice pulse h1 pos 10 1200');
    processLine('@SERVICE_INFO fw=step9j build=READONLY audit=1 diagnosticLock=1 auto=0');
    $('auditAllBtn').click();$('auditAllBtn').click();
    processLine('@AUDIT state=START scan=1 rows=93 drives=4 fc=03 writes=0 motionPermit=0');
    processLine('@AUDIT_ROW scan=1 drive=0 addr=1 key=P0.02 reg=0xF002 ok=1 value=2 error=0 exception=0 ms=100');
    processLine('@AUDIT_ROW scan=1 drive=1 addr=2 key=P0.02 reg=0xF002 ok=0 error=3 exception=2 ms=101');
    processLine('@AUDIT_ROW scan=0 drive=2 addr=3 key=P0.02 reg=0xF002 ok=1 value=2 error=0 exception=0 ms=102');
    const received=audit.received;
    processLine('@AUDIT state=DONE scan=1 errors=1');
    const incomplete=!audit.complete;
    const missing=audit.rows.get('P0.02')[1];
    const text=document.querySelector('#auditTable tr[data-key="P0.02"]').textContent;
    const pulseBlocked=document.querySelector('button[data-pulse-drive="H1"]').disabled;
    processLine('@AUDIT state=START scan=2 rows=93 drives=4 fc=03 writes=0 motionPermit=0');
    const cleared=audit.rows.size===0;
    processLine('@AUDIT state=CANCELLED scan=2');
    resetDiagnosticSession();
    const reset=diagnosticLock&&!auditSupported&&state.gates.confirmPos===0&&state.gates.confirmNeg===0;
    return {commands,oldBlocked,received,incomplete,missing,text,pulseBlocked,cleared,reset};
  });
  assert.deepEqual(result.commands,['he200 audit all\n']);assert(result.oldBlocked);
  assert.equal(result.received,2);assert(result.incomplete);assert.equal(result.missing.value,null);
  assert(result.text.includes('НЕИЗВЕСТНО'));assert(result.pulseBlocked&&result.cleared&&result.reset);
  assert.deepEqual(errors,[]);
  // Complete synthetic snapshot exercises comparison, expected values and raw DI.
  await page.evaluate(()=>{
    port={};processServiceInfo('@SERVICE_INFO fw=step9j build=READONLY audit=1 diagnosticLock=1 auto=0');
    processLine('@AUDIT state=START scan=3 rows=93 drives=4 fc=03 writes=0 motionPermit=0');
    for(const meta of HE200_AUDIT_MAP)for(let i=0;i<4;i++){
      let value=meta.expected??0;if(meta.name==='DI')value=0x4030;
      if(meta.name==='P8.13'&&i===2)value=1;
      processLine(`@AUDIT_ROW scan=3 drive=${i} addr=${i+1} key=${meta.name} reg=0x${meta.reg.toString(16)} ok=1 value=${value} error=0 exception=0 ms=1000`);
    }
    processLine('@AUDIT state=DONE scan=3 errors=0');
  });
  assert.equal(await page.evaluate(()=>audit.complete),true);
  assert((await page.locator('#auditFindings').textContent()).includes('активность не доказана'));
  assert((await page.locator('#auditTable tr[data-key="P8.13"]').textContent()).includes('РАЗЛИЧАЮТСЯ'));
  assert.equal(await page.locator('#stopAllBtn').isDisabled(),true);
  if(process.env.TEST_SCREENSHOT)await page.screenshot({path:path.resolve(process.env.TEST_SCREENSHOT),fullPage:false});
  console.log('PASS: DOM one-click/one-handler; undefined/RUN/probe/reset blocked; all-four audit; incomplete/error/stale-session handling; comparison; zero browser exceptions');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
