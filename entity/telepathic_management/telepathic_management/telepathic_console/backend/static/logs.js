(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const body = $('entries');
  let timer;
  async function refresh() {
    $('status').textContent = 'Loading…';
    try {
      const response = await fetch(`/api/logs?limit=500&level=${encodeURIComponent($('level').value)}`);
      if (!response.ok) throw new Error(`Log request failed (${response.status})`);
      const data = await response.json();
      body.replaceChildren();
      for (const entry of data.entries) {
        const row = document.createElement('tr');
        for (const value of [new Date(entry.timestamp).toLocaleString(), entry.level, entry.node, entry.message]) {
          const cell = document.createElement('td'); cell.textContent = String(value ?? ''); row.append(cell);
        }
        row.className = `level-${String(entry.level).toLowerCase()}`;
        body.append(row);
      }
      $('logPath').textContent = `Saved in ${data.path}`;
      $('status').textContent = `${data.entries.length} recent entries`;
    } catch (error) { $('status').textContent = error.message; }
  }
  function schedule() { clearInterval(timer); const seconds = Number($('refresh').value); if (seconds) timer = setInterval(refresh, seconds * 1000); }
  $('reload').addEventListener('click', refresh);
  $('level').addEventListener('change', refresh);
  $('refresh').addEventListener('change', schedule);
  refresh(); schedule();
})();
