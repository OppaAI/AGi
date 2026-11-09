/* Poll the local Web Teleops Console status so the shared UI can reflect the AIVA PC heartbeat. */
(() => {
  'use strict';
  const label = document.getElementById('aivaPresence');
  let stopped = false;
  let aivaOnline = false;
  let auroraOnline = false;

  function updateLabel() {
    if (!label) return;
    if (aivaOnline && auroraOnline) label.textContent = 'AIVA + AuRoRA online';
    else if (aivaOnline) label.textContent = 'AIVA online';
    else if (auroraOnline) label.textContent = 'AuRoRA online';
    else label.textContent = 'Offline';
  }

  async function refresh() {
    try {
      const response = await fetch('/api/presence', { cache: 'no-store' });
      if (!response.ok) throw new Error(`Presence request returned ${response.status}`);
      const status = await response.json();
      aivaOnline = status.aiva_online === true;
      auroraOnline = status.aurora_online === true;
      updateLabel();
      window.dispatchEvent(new CustomEvent('agi-aiva-presence', { detail: { online: aivaOnline } }));
      window.dispatchEvent(new CustomEvent('agi-aurora-presence', { detail: { online: auroraOnline } }));
    } catch (error) {
      aivaOnline = false;
      auroraOnline = false;
      updateLabel();
      window.dispatchEvent(new CustomEvent('agi-aiva-presence', { detail: { online: false } }));
      window.dispatchEvent(new CustomEvent('agi-aurora-presence', { detail: { online: false } }));
      console.debug('AIVA presence is not available:', error);
    }
    if (!stopped) window.setTimeout(refresh, 4000);
  }

  window.addEventListener('pagehide', () => { stopped = true; }, { once: true });
  refresh();
})();
