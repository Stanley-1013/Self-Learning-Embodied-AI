import type {ReactNode} from 'react';
import {useCallback} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './LocaleToggle.module.css';

const SCROLL_KEY = 'locale-switch-scroll';

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
  const {i18n, siteConfig} = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;

  const isDefault = i18n.currentLocale === i18n.defaultLocale;
  const targetLocale = isDefault ? 'en' : i18n.defaultLocale;
  const targetLabel = isDefault ? 'EN' : '中';

  // Scroll restoration is handled by a blocking <script> in <head>
  // (docusaurus.config.ts headTags) — no flash, runs before first paint.

  const handleClick = useCallback(() => {
    const scrollRatio = document.body.scrollHeight > 0
      ? window.scrollY / document.body.scrollHeight
      : 0;
    try {
      sessionStorage.setItem(SCROLL_KEY, JSON.stringify({scrollRatio, timestamp: Date.now()}));
    } catch {
      // ignore
    }

    // Read from window.location to avoid router-level pathname ambiguity
    const fullPath = window.location.pathname;
    const baseClean = baseUrl.replace(/\/$/, '');

    // Strip baseUrl prefix
    let afterBase = fullPath.startsWith(baseClean)
      ? fullPath.slice(baseClean.length)
      : fullPath;
    if (!afterBase.startsWith('/')) {
      afterBase = '/' + afterBase;
    }

    // Strip any non-default locale prefix (handles ALL non-default locales, not just /en/)
    const nonDefaultLocales = i18n.locales.filter((l) => l !== i18n.defaultLocale);
    let pathWithoutLocale = afterBase;
    for (const locale of nonDefaultLocales) {
      const prefix = `/${locale}`;
      if (afterBase === prefix) {
        pathWithoutLocale = '/';
        break;
      }
      if (afterBase.startsWith(prefix + '/')) {
        pathWithoutLocale = afterBase.slice(prefix.length);
        break;
      }
    }

    const targetPath = targetLocale === i18n.defaultLocale
      ? `${baseClean}${pathWithoutLocale}`
      : `${baseClean}/${targetLocale}${pathWithoutLocale}`;

    window.location.href = targetPath || '/';
  }, [baseUrl, i18n.locales, i18n.defaultLocale, targetLocale]);

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
