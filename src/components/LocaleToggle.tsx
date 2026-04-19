import type {ReactNode} from 'react';
import {useCallback} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useLocation} from '@docusaurus/router';
import styles from './LocaleToggle.module.css';

const SCROLL_KEY = 'locale-switch-scroll';
const SCROLL_TTL_MS = 5000;

/**
 * Floating language toggle (FAB-style).
 *
 * Saves scroll position before navigating, restores it on the target page
 * so the user lands at the exact same spot — convenient for comparing
 * translations side by side.
 *
 * Rendered via `src/theme/Root.tsx` on every page.
 */
export default function LocaleToggle(): ReactNode {
  const {i18n} = useDocusaurusContext();
  const {pathname} = useLocation();

  const currentLocale = i18n.currentLocale;
  const isDefault = currentLocale === i18n.defaultLocale;
  const targetLocale = isDefault ? 'en' : i18n.defaultLocale;
  const targetLabel = isDefault ? 'EN' : '中';

  // Scroll restoration is handled by a blocking <script> in <head>
  // (docusaurus.config.ts headTags) — no flash, runs before first paint.

  const handleClick = useCallback(() => {
    // Save current scroll ratio (position / total height)
    const scrollRatio = document.body.scrollHeight > 0
      ? window.scrollY / document.body.scrollHeight
      : 0;
    try {
      sessionStorage.setItem(SCROLL_KEY, JSON.stringify({scrollRatio, timestamp: Date.now()}));
    } catch {
      // ignore
    }

    let targetPath: string;
    if (isDefault) {
      targetPath = `/en${pathname}`;
    } else {
      targetPath = pathname.replace(/^\/en(\/|$)/, '/');
    }
    window.location.href = targetPath;
  }, [pathname, isDefault]);

  return (
    <button
      type="button"
      className={styles.fab}
      onClick={handleClick}
      aria-label={`Switch to ${targetLocale === 'en' ? 'English' : '繁體中文'}`}
      title={`Switch to ${targetLocale === 'en' ? 'English' : '繁體中文'}`}>
      <span className={styles.icon} aria-hidden="true">🌐</span>
      <span className={styles.label}>{targetLabel}</span>
    </button>
  );
}
