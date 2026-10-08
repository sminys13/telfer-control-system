const assert=require('node:assert/strict');
const path=require('node:path');
const {pathToFileURL}=require('node:url');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1050}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto(pathToFileURL(path.resolve('tools/telfer_web_control/index.html')).href);
  assert.deepEqual(errors,[]);assert.equal(await page.locator('#stop').isDisabled(),true);
  await page.evaluate(()=>{globalThis.testCommands=[];port={};writer={write:async data=>testCommands.push(new TextDecoder().decode(data))};processLine('@SERVICE_INFO fw=v6-system-step9i-he200-service-cockpit build=FIELD diagnosticLock=0 auto=0');});
  assert.equal(await page.locator('#arm').isDisabled(),true);
  await page.reload();
  await page.evaluate(()=>{
   globalThis.testCommands=[];port={};writer={write:async data=>{
    const command=new TextDecoder().decode(data).trim();testCommands.push(command);
    const match=command.match(/ @(\d+)$/),id=match?.[1];
    if(!id)return;
    queueMicrotask(()=>{
     if(command.startsWith('web read ')){
      const parts=command.split(' '),key=parts[3],value=key==='P0.22'||key==='P0.19'?key==='P0.22'?2:1:key==='P6.03'?0:30;
      processLine(`@WEB_PARAM id=${id} drive=${parts[2]} key=${key} ok=1 raw=${value} error=0 exception=0`);
     }else processLine(`@WEB_ACK id=${id} ok=1`);
    });
   }};
   processLine('@SERVICE_INFO fw=v6-system-step9k-web-control build=FIELD diagnosticLock=0 auto=1');
   processLine('@WEB_STATE session=42 armed=0 cal=0 running=0 paused=0 pulse=0 ready=1 step=0 steps=20 estop=0 limitsEnabled=0');
   for(let i=0;i<4;i++)processLine(`@WEB_AXIS i=${i} mm=1000 raw=1000 age=10 fwdSign=-1 max=40 manual=10 slow=10 band=100 tolerance=8 invert=0 offset=0`);
   processLine('@PROGRAM slot=1 zones=2 home1=1000 home2=1000 travel1=1000 travel2=1000 drip=2 tiltPct=10 lowSide=0');
   processLine('@ZONE n=1 en=1 x1=1000 x2=1000 z1=900 z2=900 dip=2 tilt=0 wait=0 hpct=10 vpct=10');
   processLine('@ZONE n=2 en=0 x1=1050 x2=1050 z1=900 z2=900 dip=2 tilt=0 wait=0 hpct=15 vpct=20');
   processLine('@WEB_ORDER values=2,1');
   processLine('@WEB_DRY enabled=0 seconds=60 staging=1 x1=1000 x2=1000 z1=900 z2=900');
  });
  assert.equal(await page.locator('#arm').isDisabled(),true);
  await page.locator('#external').check();assert.equal(await page.locator('#arm').isDisabled(),false);
  await page.evaluate(()=>processLine('@WEB_STATE session=42 armed=1 cal=3 running=0 paused=0 pulse=0 ready=1 step=0 steps=20 estop=0 limitsEnabled=0'));
  assert.equal(await page.locator('[data-jog="h"][data-dir="pos"]').isDisabled(),false);
  assert.equal(await page.locator('[data-jog="v"][data-dir="pos"]').isDisabled(),true);
  assert.equal(await page.locator('#startAuto').isDisabled(),true);
  // Opening an AVR serial port may reset it after the first query was sent.
  await page.evaluate(()=>processLine('FW: v6-system-step9k-web-control'));
  await page.waitForTimeout(30);
  assert((await page.evaluate(()=>testCommands)).includes('service info'));
  assert.equal(await page.locator('#arm').isDisabled(),true);
  await page.evaluate(()=>{
   processLine('@SERVICE_INFO fw=v6-system-step9k-web-control build=FIELD diagnosticLock=0 auto=1');
   processLine('@WEB_STATE session=43 armed=0 cal=0 running=0 paused=0 pulse=0 ready=1 step=0 steps=20 estop=0 limitsEnabled=0');
   processLine('@WEB_STATE session=43 armed=1 cal=15 running=0 paused=0 pulse=0 ready=1 step=0 steps=20 estop=0 limitsEnabled=0');
   controller.session='42';
   for(let i=0;i<4;i++)processLine(`@WEB_AXIS i=${i} mm=1000 raw=1000 age=10 fwdSign=-1 max=40 manual=10 slow=10 band=100 tolerance=8 invert=0 offset=0`);
  });
  await page.evaluate(()=>processLine('@WEB_STATE session=42 armed=1 cal=15 running=0 paused=0 pulse=0 ready=1 step=0 steps=20 estop=0 limitsEnabled=0'));
  assert.equal(await page.locator('#startAuto').isDisabled(),false);
  assert.equal(await page.locator('#order').inputValue(),'2 1');
  assert.equal(await page.locator('#zones tr').nth(1).locator('[data-key="enabled"]').isChecked(),false);
  assert.equal(await page.locator('#zones tr').nth(1).locator('[data-key="hPct"]').inputValue(),'15');
  assert.equal(await page.locator('#zones tr').nth(1).locator('[data-key="vPct"]').inputValue(),'20');
  const button=page.locator('[data-jog="h1"][data-dir="pos"]'),bounds=await button.boundingBox();
  await page.mouse.move(bounds.x+bounds.width/2,bounds.y+bounds.height/2);await page.mouse.down();await page.waitForTimeout(90);await page.mouse.up();await page.waitForTimeout(50);
  const jogCommands=await page.evaluate(()=>testCommands.filter(x=>x.startsWith('web jog ')));
  assert.equal(jogCommands.length,1);assert(jogCommands[0].includes('h1 pos 10'));assert((await page.evaluate(()=>testCommands)).some(x=>x==='web release 42'));
  assert(!jogCommands.some(x=>x.includes('undefined')));
  // IDs prevent an unrelated acknowledgement from completing a request.
  const correlation=await page.evaluate(async()=>{
   const sent=[];writer={write:async data=>sent.push(new TextDecoder().decode(data))};const p=request('web info');const id=seq;let resolved=false;p.then(()=>resolved=true);
   processLine(`@WEB_ACK id=${id+100} ok=1`);await Promise.resolve();const blocked=!resolved;
   await new Promise(resolve=>setTimeout(resolve,350));const noHeartbeat=sent.every(c=>!c.startsWith('web ping '));
   processLine(`@WEB_ACK id=${id} ok=1`);await p;return blocked&&noHeartbeat;
  });assert(correlation);
  await page.evaluate(()=>{writer={write:async data=>{const c=new TextDecoder().decode(data).trim();testCommands.push(c);const id=c.match(/ @(\d+)$/)?.[1];if(id)queueMicrotask(()=>processLine(`@WEB_ACK id=${id} ok=1`));}};});
  await page.locator('[data-tab="program"]').click();
  await page.locator('#home1').fill('1001');await page.locator('#home1').dispatchEvent('change');assert.equal(await page.locator('#startAuto').isDisabled(),true);
  const planned=await page.evaluate(()=>collectProgram());assert.equal(planned.home1,1001);assert.deepEqual(planned.order,[2,1]);assert.equal(planned.zones[1].enabled,0);
  await page.locator('#addZone').click();assert.equal(await page.locator('#order').inputValue(),'2 1 3');
  await page.locator('#removeZone').click();assert.equal(await page.locator('#order').inputValue(),'2 1');
  await page.locator('#applyProgram').click();await page.waitForTimeout(150);
  const commands=await page.evaluate(()=>testCommands);
  assert(commands.some(x=>x.startsWith('web program 42 count 2')));assert(commands.some(x=>x.startsWith('web zone 42 2 1050 1050 900 900 2 0 0 15 20 0')));assert(commands.some(x=>x.startsWith('web program 42 order 2 1')));
  // Update status without trampling unsaved settings.
  await page.evaluate(()=>{const card=document.querySelector('[data-profile-card="h1"]');card.dataset.dirty='1';card.querySelector('[data-key="max"]').value=35;processLine('@WEB_AXIS i=0 mm=1001 raw=1001 age=0 fwdSign=-1 max=40 manual=10 slow=10 band=100 tolerance=8 invert=0 offset=0');});
  assert.equal(await page.locator('[data-profile-card="h1"] [data-key="max"]').inputValue(),'35');
  await page.evaluate(()=>{parameters.set('v1:P0.22',{ok:'1',raw:'2'});parameters.set('v1:P6.03',{ok:'1',raw:'1000'});renderParameter({drive:'v1',key:'P6.03',ok:'1',raw:'1000'});});
  assert.equal(await page.locator('[data-param="P6.03"][data-drive="v1"] input').inputValue(),'10');
  const emptyBlocked=await page.evaluate(async()=>{const cell=document.querySelector('[data-param="P6.03"][data-drive="v1"]');cell.querySelector('input').value='';try{await writeParameter(cell.querySelector('button'));return false;}catch{return true;}});assert(emptyBlocked);
  await page.locator('[data-tab="manual"]').click();
  if(process.env.TEST_SCREENSHOT)await page.screenshot({path:path.resolve(process.env.TEST_SCREENSHOT),fullPage:true});
  // STOP invalidates queued stale commands and disarms local controls immediately.
  await page.locator('#stop').click();assert.equal(await page.locator('[data-jog="h1"][data-dir="pos"]').isDisabled(),true);
  assert.deepEqual(errors,[]);
  await page.setViewportSize({width:600,height:900});if(process.env.TEST_MOBILE_SCREENSHOT)await page.screenshot({path:path.resolve(process.env.TEST_MOBILE_SCREENSHOT),fullPage:true});
  console.log('PASS: actual Step9K DOM; incompatible firmware blocked; arm/direction gates; one pointerdown/one command; release; correlated ACK; zone/order editing; parameters and unknown/empty values; draft preservation; STOP; zero page errors; desktop/mobile layouts');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
