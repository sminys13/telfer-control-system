const assert = require('node:assert/strict');
const path = require('node:path');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const {pathToFileURL} = require('node:url');
(async () => {
  const browser = await chromium.launch({channel: 'msedge', headless: true});
  try {
    const page = await browser.newPage({viewport: {width: 1500, height: 1100}}), errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.goto(process.env.TEST_URL || pathToFileURL(path.resolve('tools/telfer_web_control/index.html')).href);
    assert.deepEqual(errors, []);assert.equal(await page.locator('#stop').isDisabled(), true);
    // Offline editing needs neither ARM nor a hardware connection.
    await page.locator('[data-tab="program"]').click();
    await page.locator('#zoneCount').fill('4');await page.locator('#resizeZones').click();assert.equal(await page.locator('#zones tr').count(), 4);
    await page.locator('#zones tr').nth(1).locator('[data-move="-1"]').click();assert.deepEqual(await page.evaluate(() => collectProgram().order), [2, 1, 3, 4]);
    await page.locator('#duplicateZone').click();assert.equal(await page.locator('#zones tr').count(), 5);
    await page.locator('#removeZone').click();assert.equal(await page.locator('#zones tr').count(), 4);
    await page.evaluate(() => {port = {};writer = {write: async () => {}};processLine('@SERVICE_INFO fw=v6-system-step9k-web-control build=FIELD diagnosticLock=0 auto=1 dwinMotion=0');});
    assert.equal(await page.locator('#arm').isDisabled(), true);
    await page.reload();
    await page.evaluate(() => {
      globalThis.testCommands = [];globalThis.testProgram = ProgramModel.demo();globalThis.testState = {session: '42', armed: '0', cal: '15', running: '0', paused: '0', pulse: '0', ready: '1', step: '0', steps: '20', estop: '0', slot: '1', phase: 'IDLE', issue: '0', desktopSim: '1', physicalTx: '0', limitsEnabled: '0'};
      globalThis.sendState = () => processLine('@WEB_STATE ' + Object.entries(testState).map(([k, v]) => `${k}=${v}`).join(' '));
      globalThis.feedProgram = () => {
        const p = testProgram;processLine(`@WEB_SLOT slot=${p.slot} nameHex=${encodeName(p.name)}`);
        processLine(`@PROGRAM slot=${p.slot} zones=${p.zones.length} home1=${p.home1} home2=${p.home2} travel1=${p.travel1} travel2=${p.travel2} drip=${p.drip} tiltPct=${p.tiltPct} lowSide=${p.lowSide}`);
        p.zones.forEach((z, i) => processLine(`@ZONE n=${i + 1} en=${z.enabled} x1=${z.x1} x2=${z.x2} z1=${z.z1} z2=${z.z2} dip=${z.dip} tilt=${z.tilt} wait=${z.wait} hpct=${z.hPct} vpct=${z.vPct}`));
        processLine(`@WEB_ORDER values=${p.order.join(',')}`);const d = p.drying;processLine(`@WEB_DRY enabled=${d.enabled} seconds=${d.seconds} staging=${d.staging} x1=${d.x1} x2=${d.x2} z1=${d.z1} z2=${d.z2}`);processLine('@WEB_PROGRAM_END');
      };
      port = {};writer = {write: async data => {
        const c = new TextDecoder().decode(data).trim();testCommands.push(c);const id = c.match(/ @(\d+)$/)?.[1], t = c.replace(/ @\d+$/, '').split(' ');
        if (c.startsWith('web arm ')) {testState.armed = '1';sendState();}
        if (c === 'web stop') {testState.armed = '0';testState.running = '0';sendState();}
        if (c.startsWith('web program ') && t[3] === 'count') {testProgram.zones = testProgram.zones.slice(0, Number(t[4]));testProgram.order = testProgram.zones.map((_, i) => i + 1);}
        if (c.startsWith('web program ') && ['home', 'travel'].includes(t[3])) {testProgram[t[3] + '1'] = Number(t[4]);testProgram[t[3] + '2'] = Number(t[5]);}
        if (c.startsWith('web program ') && t[3] === 'name') testProgram.name = decodeName(t[4]);
        if (c.startsWith('web program ') && t[3] === 'order') testProgram.order = t.slice(4).map(Number);
        if (c.startsWith('web zone ')) {const [, , , n, x1, x2, z1, z2, dip, tilt, wait, hPct, vPct, enabled] = t.map((v, i) => i < 3 ? v : Number(v));testProgram.zones[n - 1] = {n, x1, x2, z1, z2, dip, tilt, wait, hPct, vPct, enabled};}
        queueMicrotask(() => {
          if (c.startsWith('web info')) feedProgram();
          if (id) {
            if (c.startsWith('web read ')) {const key = t[3], value = key === 'P0.22' ? 2 : key === 'P0.19' ? 1 : key === 'P6.03' ? 0 : 30;processLine(`@WEB_PARAM id=${id} drive=${t[2]} key=${key} ok=1 raw=${value} error=0 exception=0`);}
            else processLine(`@WEB_ACK id=${id} ok=1`);
          }
        });
      }};
      forceReload = true;processLine('@SERVICE_INFO fw=v6-system-step9l-web-control build=SIM diagnosticLock=1 auto=0 dwinMotion=0 desktopSim=1 physicalTx=0');sendState();
      for (let i = 0; i < 4; i++) processLine(`@WEB_AXIS i=${i} mm=1000 raw=1000 valid=1 hwOk=1 age=10 fwdSign=-1 max=40 manual=10 slow=10 band=100 tolerance=8 invert=0 offset=0`);
    });
    await page.waitForTimeout(80);
    assert.equal(await page.locator('#externalLabel').isVisible(), false);assert.equal(await page.locator('#arm').isDisabled(), false);
    await page.locator('#arm').click();await page.waitForTimeout(60);assert.equal(await page.locator('#startAuto').isDisabled(), false);
    await page.locator('[data-tab="manual"]').click();
    const b = page.locator('[data-jog="h1"][data-dir="pos"]'), box = await b.boundingBox();await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);await page.mouse.down();await page.waitForTimeout(85);await page.mouse.up();await page.waitForTimeout(40);
    const commands = await page.evaluate(() => testCommands);assert.equal(commands.filter(c => c.startsWith('web jog ')).length, 1);assert(commands.includes('web release 42'));assert(!commands.some(c => c.includes('undefined')));
    await page.locator('[data-tab="program"]').click();await page.locator('#home1').fill('1001');await page.locator('#home1').dispatchEvent('change');assert.equal(await page.locator('#startAuto').isDisabled(), true);
    await page.locator('#programName').fill('Проверка');await page.locator('#programName').dispatchEvent('change');
    await page.locator('#zones tr').nth(1).locator('[data-move="-1"]').click();
    await page.locator('#onlySelectedZone').click();assert.equal((await page.evaluate(()=>collectProgram())).zones.filter(z=>z.enabled).length,1);await page.locator('#enableAllZones').click();
    await page.locator('#applyProgram').click();await page.waitForTimeout(140);
    const tx = await page.evaluate(() => testCommands);assert(tx.some(c => c.startsWith('web program 42 order 2 1')));assert(tx.some(c => c.startsWith('web program 42 commit')));assert(tx.some(c => c.startsWith('web program 42 name ')));
    assert.equal(await page.locator('#startAuto').isDisabled(), false);
    await page.evaluate(() => {const card = document.querySelector('[data-profile-card="h1"]');card.dataset.dirty = '1';card.querySelector('[data-key="max"]').value = 35;processLine('@WEB_AXIS i=0 mm=1001 raw=1001 valid=1 hwOk=1 age=0 fwdSign=-1 max=40 manual=10 slow=10 band=100 tolerance=8 invert=0 offset=0');});
    assert.equal(await page.locator('[data-profile-card="h1"] [data-key="max"]').inputValue(), '35');
    const correlated = await page.evaluate(async () => {const sent = [];writer = {write: async data => sent.push(new TextDecoder().decode(data))};const p = request('web info'), id = seq;let done = false;p.then(() => done = true);processLine(`@WEB_ACK id=${id + 10} ok=1`);await new Promise(r => setTimeout(r, 350));const blocked = !done && sent.every(c => !c.startsWith('web ping '));processLine(`@WEB_ACK id=${id} ok=1`);await p;return blocked;});assert(correlated);
    await page.evaluate(() => {writer = {write: async data => {const c = new TextDecoder().decode(data).trim();testCommands.push(c);if (c === 'web stop') {testState.armed = '0';sendState();}}};});
    // STOP aborts the transfer promise; the remaining fields are never sent.
    await page.locator('#home1').fill('1002');await page.locator('#home1').dispatchEvent('change');const start = await page.evaluate(() => testCommands.length);await page.locator('#applyProgram').click();await page.waitForTimeout(50);await page.locator('#stop').click();await page.waitForTimeout(70);
    const cancelled = await page.evaluate(n => testCommands.slice(n), start);assert.equal(cancelled.filter(c => c.startsWith('web program ')).length, 1);assert(cancelled.includes('web stop'));
    await page.locator('[data-tab="overview"]').click();await page.evaluate(() => {testState.armed = '0';testState.phase = 'DONE';testState.step = '20';testState.zone = '1';sendState();processLine('@WEB_TIME elapsed=32 timed=4');processLine('@WEB_MEMORY free=1200 minimum=900 tx1=0 spi=0');});
    await page.evaluate(()=>window.scrollTo(0,0));await page.waitForTimeout(350);
    if (process.env.TEST_SCREENSHOT) await page.screenshot({path: path.resolve(process.env.TEST_SCREENSHOT), fullPage: true});
    await page.locator('[data-tab="program"]').click();await page.evaluate(()=>window.scrollTo(0,0));if (process.env.TEST_EDITOR_SCREENSHOT) await page.screenshot({path: path.resolve(process.env.TEST_EDITOR_SCREENSHOT), fullPage: true});
    await page.setViewportSize({width: 600, height: 900});await page.locator('[data-tab="overview"]').click();if (process.env.TEST_MOBILE_SCREENSHOT) await page.screenshot({path: path.resolve(process.env.TEST_MOBILE_SCREENSHOT), fullPage: true});
    assert.deepEqual(errors, []);
    console.log('PASS: offline table edit/reorder/clone/delete; strict firmware modes; SIM arm; one jog/release; correlated ACK/no heartbeat spam; UTF-8 name and transfer commit; dirty drafts; STOP cancels transfer; dashboard/mobile; zero JS errors');
  } finally {await browser.close();}
})().catch(e => {console.error(e);process.exitCode = 1;});
