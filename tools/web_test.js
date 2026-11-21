/* 模拟浏览器环境，验证调参页的滑块不会被状态轮询弹回。
 * 只验证逻辑，不渲染样式。
 */
const fs = require('fs');
const vm = require('vm');

const html = fs.readFileSync('<仓库目录>/rx_uac/main/index.html', 'utf8');
const src  = html.match(/<script>([\s\S]*?)<\/script>/)[1];

/* ---- 极简 DOM ---- */
const elements = {};
function mkEl(id) {
  return {
    id, value: '0', min: '0', max: '100',
    textContent: '', innerHTML: '', onclick: null,
    style: { setProperty() {}, color: '' },
    _h: {},
    addEventListener(t, fn) { (this._h[t] = this._h[t] || []).push(fn); },
    fire(t) { (this._h[t] || []).forEach(fn => fn()); },
  };
}
['s-tq','s-db','s-g','s-lim','v-tq','v-db','v-g','v-lim',
 'v-q','v-ratio','v-lat','v-pkt','v-rssi','v-lost','v-under','v-pad',
 'v-plc','v-over','v-clamp','v-jump','v-zero','v-rate','v-out',
 'sub','b-apply','b-reset'].forEach(id => { elements[id] = mkEl(id); });

/* 滑块的取值范围，和 HTML 保持一致 */
Object.assign(elements['s-tq'],  { min:'8',  max:'40', step:'1'   });
Object.assign(elements['s-db'],  { min:'0.5',max:'8',  step:'0.1' });
Object.assign(elements['s-g'],   { min:'2',  max:'32', step:'1'   });
Object.assign(elements['s-lim'], { min:'50', max:'400',step:'10'  });

/* ---- 服务端状态 ---- */
const server = { tq: 18, db: 3.0, g: 8, lim: 200 };
const requests = [];

function stateJson() {
  return {
    q: server.tq, qmin: server.tq, qmax: server.tq, ratio: 0.0,
    rate: 200, rssi: -50, lost: 0, lostpct: 0, under: 0, pad: 0, plc: 0,
    over: 0, clamp: 0, jumpMax: 0, zero: 0, i2srate: 47994, outrate: 47996.1,
    dbfs: -20, tune: { ...server },
  };
}

/* ---- 假定时器 ---- */
let timers = [];
const sandbox = {
  console,
  Date,
  Math,
  Number,
  String,
  Object,
  Array,
  JSON,
  setTimeout(fn, ms) { timers.push({ fn, ms }); return timers.length; },
  clearTimeout(id) { if (id) timers[id - 1] = null; },
  setInterval() { return 0; },
  fetch(url) {
    requests.push(url);
    if (url.startsWith('/api/set?')) {
      url.split('?')[1].split('&').forEach(kv => {
        const [k, v] = kv.split('=');
        server[k] = Number(v);
      });
    }
    return Promise.resolve({ ok: true, json: () => Promise.resolve(stateJson()) });
  },
};
sandbox.document = { getElementById: id => elements[id] || (elements[id] = mkEl(id)) };
sandbox.globalThis = sandbox;

vm.createContext(sandbox);
vm.runInContext(
  src + '\n;globalThis.__x = { poll, apply, get dirty(){return dirty}, get editSeq(){return editSeq} };',
  sandbox
);
const X = sandbox.__x;

const flush = () => { const t = timers; timers = []; t.forEach(o => o && o.fn()); };
const tick  = () => new Promise(r => setImmediate(r));
let failed = 0;
function ok(cond, name, extra) {
  console.log((cond ? '  PASS  ' : '  FAIL  ') + name + (cond ? '' : '   <<< ' + extra));
  if (!cond) failed++;
}

(async () => {
  await tick();                       /* 首轮 poll */
  ok(elements['s-tq'].value == 18, '首屏滑块跟随服务端值 18', elements['s-tq'].value);

  /* 场景一：用户把 TARGET_Q 拖到 25，下一轮轮询必须不把它弹回 18 */
  elements['s-tq'].value = '25';
  elements['s-tq'].fire('input');
  ok(X.dirty === true, '拖动后标记 dirty', X.dirty);

  X.poll();                           /* 模拟 800ms 轮询插进来 */
  await tick();
  ok(elements['s-tq'].value == '25', '轮询不覆盖用户拖出的值（原 bug 在这里）', elements['s-tq'].value);

  /* 场景二：松手 300ms 后自动下发 */
  flush();
  await tick(); await tick();
  ok(requests.some(u => u.includes('tq=25')), '自动下发了 tq=25', requests.join(' | '));
  ok(server.tq === 25, '服务端已收到 25', server.tq);
  ok(X.dirty === false, '落地后清除 dirty', X.dirty);

  /* 场景三：连续拖两次，只有最后一次的值留下 */
  elements['s-tq'].value = '20'; elements['s-tq'].fire('input');
  elements['s-db'].value = '4.5'; elements['s-db'].fire('input');
  flush();
  await tick(); await tick();
  X.poll(); await tick();
  ok(server.tq === 20 && server.db === 4.5, '两个滑块的值都提交了', JSON.stringify(server));
  ok(elements['s-tq'].value == 20 && elements['s-db'].value == 4.5, '提交后滑块保持用户值', elements['s-tq'].value + '/' + elements['s-db'].value);

  /* 场景四：恢复默认回到首次拉到的固件值（18/3/8/200） */
  elements['b-reset'].onclick();
  await tick(); await tick();
  ok(server.tq === 18 && server.db === 3 && server.g === 8 && server.lim === 200,
     '恢复默认写回 18/3/8/200', JSON.stringify(server));
  ok(elements['s-tq'].value == 18, '滑块显示恢复后的值', elements['s-tq'].value);

  /* 场景五：请求失败不能把 dirty 清掉，否则用户改动会被悄悄丢弃 */
  sandbox.fetch = () => Promise.reject(new Error('net down'));
  elements['s-g'].value = '24'; elements['s-g'].fire('input');
  flush();
  await tick(); await tick();
  ok(X.dirty === true, '失败时保留 dirty（不丢用户改动）', X.dirty);
  ok(elements['s-g'].value == '24', '失败时滑块保持 24', elements['s-g'].value);

  console.log(failed ? `\n${failed} 项失败` : '\n全部通过');
  process.exit(failed ? 1 : 0);
})();
