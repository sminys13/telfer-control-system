const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {pathToFileURL}=require('node:url');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
(async()=>{
 const measured=JSON.parse(fs.readFileSync('test/diagnostic/fixtures/he200-real-audit-20261008.json','utf8'));
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1100}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto(pathToFileURL(path.resolve('tools/telfer_service_console/index.html')).href);
  const findings=await page.evaluate(data=>{
   port={};processLine('@SERVICE_INFO fw=v6-system-step9j2-he200-readonly-audit build=READONLY audit=1 diagnosticLock=1 auto=0');
   processLine('@AUDIT state=START scan=1 rows=93 drives=4 fc=03 writes=0 motionPermit=0');
   for(const meta of HE200_AUDIT_MAP)for(let i=0;i<4;i++)
    processLine(`@AUDIT_ROW scan=1 drive=${i} addr=${i+1} key=${meta.name} reg=0x${meta.reg.toString(16)} ok=1 value=${data.values[meta.name][i]} error=0 exception=0 ms=1000`);
   processLine('@AUDIT state=DONE scan=1 errors=0');
   return {complete:audit.complete,text:$('auditFindings').textContent,received:audit.received};
  },measured);
  assert(findings.complete);assert.equal(findings.received,372);
  const lines=findings.text.split('\n');
  for(const line of lines.slice(0,2))assert(!line.includes('основной кандидат'));
  for(const line of lines.slice(2,4)){
   assert(line.includes('задание 5.00 Гц ниже стартовой частоты P6.03=10.00 Гц'));
   assert(line.includes('это не подтверждение причины по результату RUN'));
   assert(line.includes('P8.14=0 задаёт работу на нижнем пределе'));
   assert(line.includes('предвозбуждение: 30%, 0.3 с'));
   assert(line.includes('P0.18=0'));
  }
  assert(!lines[2].includes('ПЧ инвертирует направление'));
  assert(lines[3].includes('ПЧ инвертирует направление'));
  assert((await page.locator('#auditTable tr[data-key="SET_HZ"]').textContent()).includes('5.00 Гц (задание)'));
  assert((await page.locator('#auditTable tr[data-key="RUN_HZ"]').textContent()).includes('0.00 Гц (выход)'));
  assert.equal(await page.locator('#stopAllBtn').isDisabled(),true);
  for(const dir of ['pos','neg'])assert.equal(await page.locator(`button[data-pulse-drive="V1"][data-dir="${dir}"]`).isDisabled(),true);
  assert.equal(await page.locator('#auditTable tbody tr').count(),93);
  if(process.env.TEST_SCREENSHOT)await page.screenshot({path:path.resolve(process.env.TEST_SCREENSHOT),fullPage:false});
  // Unknown frequency resolution must not silently produce a diagnosis.
  const unknown=await page.evaluate(()=>{
   audit.rows.get('P0.22')[2]={ok:false,value:null,error:3,exception:2};
   auditFindings();return {hz:parameterFrequencyHz('P6.03',2),line:$('auditFindings').textContent.split('\n')[2]};
  });assert.equal(unknown.hz,null);assert(!unknown.line.includes('основной кандидат'));
  assert.deepEqual(errors,[]);
  console.log('PASS: actual 372-row snapshot replay; V1/V2 startup mismatch; lower-limit mode distinguished; V2 inversion; set/output frequencies; unknown scaling; no motion controls; zero page errors');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
