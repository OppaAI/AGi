/* Web Teleops Console theme preference follows Aidon's shared light/dark theme behavior. */
(() => {
  'use strict';

  const KEY = 'aidon-theme';
  const root = document.documentElement;
  const button = document.getElementById('themeToggle');

  function readTheme() {
    try {
      return localStorage.getItem(KEY) === 'dark' ? 'dark' : 'light';
    } catch (_) {
      return 'light';
    }
  }

  function applyTheme(theme, save = false) {
    const normalized = theme === 'dark' ? 'dark' : 'light';
    root.setAttribute('data-theme', normalized);
    if (save) {
      try { localStorage.setItem(KEY, normalized); } catch (_) { /* storage can be disabled */ }
    }
    if (button) {
      const next = normalized === 'dark' ? 'light' : 'dark';
      button.setAttribute('aria-label', `Switch to ${next} theme`);
      button.querySelector('span').textContent = normalized === 'dark' ? 'Dark' : 'Light';
    }
  }

  applyTheme(readTheme());
  button?.addEventListener('click', () => {
    applyTheme(root.getAttribute('data-theme') === 'dark' ? 'light' : 'dark', true);
  });
  window.addEventListener('storage', (event) => {
    if (event.key === KEY || event.key === null) applyTheme(readTheme());
  });
})();
