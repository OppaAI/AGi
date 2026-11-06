(() => {
  'use strict';
  const form = document.getElementById('settingsForm');
  const groupsRoot = document.getElementById('settingsGroups');
  const runtimeRoot = document.getElementById('runtimeSettings');
  const state = document.getElementById('saveState');
  const saveButton = document.getElementById('saveButton');
  const original = new Map();
  let settings = [];

  const escapeHtml = (value) => String(value).replace(/[&<>"']/g, (char) => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));

  function render(data) {
    settings = data.settings || [];
    document.getElementById('configPath').textContent = `Configuration file · ${data.path}`;
    document.getElementById('applyNote').textContent = data.apply_note;
    document.getElementById('runtimeNote').textContent = data.read_only_note;
    const groups = new Map();
    for (const item of settings) {
      if (!groups.has(item.group)) groups.set(item.group, []);
      groups.get(item.group).push(item);
      original.set(item.key, item.value);
    }
    groupsRoot.innerHTML = [...groups].map(([name, items]) => `<section class="setting-group" data-group="${escapeHtml(name.toLowerCase())}"><header class="group-title"><h2>${escapeHtml(name)}</h2><span>${items.length} settings · restart to apply</span></header>${items.map((item) => `<label class="setting-row" data-search="${escapeHtml(`${item.label} ${item.key} ${item.description}`.toLowerCase())}"><span><span class="setting-label">${escapeHtml(item.label)}</span><span class="setting-description">${escapeHtml(item.description)}</span><code class="setting-key">${escapeHtml(item.key)}</code></span><input data-setting="${escapeHtml(item.key)}" type="number" min="${item.min}" max="${item.max}" step="${Number.isInteger(item.default) ? 1 : 'any'}" value="${escapeHtml(item.value)}" aria-label="${escapeHtml(item.label)}"><span class="apply-tag">↻ Restart</span></label>`).join('')}</section>`).join('');
    runtimeRoot.innerHTML = (data.runtime || []).map((item) => `<article class="runtime-card"><strong>${escapeHtml(item.name)}</strong><span>${escapeHtml(item.value)}</span><small>${escapeHtml(item.source)} · restart process</small></article>`).join('');
    state.textContent = `${settings.length} tunable settings loaded`;
  }

  async function load() {
    try {
      const response = await fetch('/api/settings', {cache: 'no-store'});
      const data = await response.json();
      if (!response.ok) throw new Error(data.detail || 'Could not load settings');
      render(data);
    } catch (error) {
      state.textContent = error.message;
      saveButton.disabled = true;
    }
  }

  document.getElementById('settingsSearch').addEventListener('input', (event) => {
    const query = event.target.value.trim().toLowerCase();
    for (const row of groupsRoot.querySelectorAll('.setting-row')) row.hidden = !row.dataset.search.includes(query);
    for (const group of groupsRoot.querySelectorAll('.setting-group')) group.hidden = !group.querySelector('.setting-row:not([hidden])');
  });
  document.getElementById('resetButton').addEventListener('click', () => {
    for (const input of form.querySelectorAll('[data-setting]')) input.value = original.get(input.dataset.setting);
    state.textContent = 'Unsaved edits reset';
  });
  form.addEventListener('input', () => { state.textContent = 'Unsaved edits'; });
  form.addEventListener('submit', async (event) => {
    event.preventDefault();
    const values = {};
    for (const input of form.querySelectorAll('[data-setting]')) {
      if (!input.reportValidity()) return;
      const meta = settings.find((item) => item.key === input.dataset.setting);
      values[input.dataset.setting] = Number.isInteger(meta.default) ? Number.parseInt(input.value, 10) : Number(input.value);
    }
    saveButton.disabled = true;
    state.textContent = 'Saving…';
    try {
      const response = await fetch('/api/settings', {method:'PUT', headers:{'Content-Type':'application/json'}, body:JSON.stringify({settings:values})});
      const result = await response.json();
      if (!response.ok) throw new Error(result.detail || 'Could not save settings');
      for (const [key, value] of Object.entries(values)) original.set(key, value);
      state.textContent = `Saved ${result.saved.length} settings · restart AuRoRA to apply`;
    } catch (error) {
      state.textContent = error.message;
    } finally {
      saveButton.disabled = false;
    }
  });
  load();
})();
