import type {ReactNode} from 'react';
import {useCallback} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useAlternatePageUtils} from '@docusaurus/theme-common/internal';
import styles from './LocaleToggle.module.css';

const SCROLL_KEY = 'locale-switch-scroll';

/**
 * Floating language toggle (FAB-style).
 *
 * Uses Docusaurus's own `useAlternatePageUtils` so path/baseUrl/locale
 * handling matches the framework's built-in LocaleDropdown exactly —
 * no ad-hoc pathname surgery.
 *
 * Scroll position is saved to sessionStorage and restored by a blocking
 * <script> injected via docusaurus.config.ts headTags — runs before
 * first paint to avoid flicker.
 *
 * Rendered via `src/theme/Root.tsx` on every page.
 */
export default function LocaleToggle(): ReactNode {
  const {i18n} = useDocusaurusContext();
  const {createUrl} = useAlternatePageUtils();

  const isDefault = i18n.currentLocale === i18n.defaultLocale;
  const targetLocale = isDefault ? 'en' : i18n.defaultLocale;
  const targetLabel = isDefault ? 'EN' : '中';

  const handleClick = useCallback(() => {
    const scrollRatio = document.body.scrollHeight > 0
      ? window.scrollY / document.body.scrollHeight
      : 0;
    try {
      sessionStorage.setItem(
        SCROLL_KEY,
        JSON.stringify({scrollRatio, timestamp: Date.now()}),
      );
    } catch {
      // ignore — sessionStorage may be unavailable
    }

    const targetUrl = createUrl({locale: targetLocale, fullyQualified: false});
    window.location.href = targetUrl;
  }, [createUrl, targetLocale]);

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
