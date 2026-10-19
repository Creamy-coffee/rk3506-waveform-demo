// 让页面进入「已连接且有数据」的假状态，用于无串口时核对实时渲染
(() => {
  const v = window.__v32;
  v.rt.connected = true;
  v.rt.t0 = 0;
  v.st.lastBuildMs = performance.now();
  v.st.startIndex = 0x100001234;
  v.st.stats = v.buildFrame({
    freqHz: 1000000, amp0: 300, amp1: 120, dc: 0,
    sigma: 8, spikes: 0, noiseType: 'gauss',
    frameId: v.st.frameId, startIndex: v.st.startIndex
  }, v.st);
  v.st.frameId += 1;
  v.st.hasFrame = true;
  v.analyse(v.st.stats.vpp0 >= v.st.stats.vpp1 ? 0 : 1);
  v.render();
  return 'fake-live ok';
})()
