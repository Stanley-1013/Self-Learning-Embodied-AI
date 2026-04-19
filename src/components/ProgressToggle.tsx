import type {ReactNode} from 'react';
import {useLocation} from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useProgressEntry} from '@site/src/lib/progress';
import {getStrings} from '@site/src/lib/strings';
import styles from './ProgressToggle.module.css';

function ToggleButton(): ReactNode {
  const {pathname} = useLocation();
  const {i18n} = useDocusaurusContext();
  const s = getStrings(i18n.currentLocale);
  const {entry, toggle} = useProgressEntry(pathname);

  const completedAt = entry.completedAt
    ? new Date(entry.completedAt).toLocaleString(s.dateLocale, {
        dateStyle: 'medium',
        timeStyle: 'short',
      })
    : null;

  const prompt = entry.completed ? s.promptDone : s.promptNotDone;
  const buttonLabel = entry.completed ? s.btnCompleted : s.btnComplete;

  return (
    <div className={styles.wrapper} role="group" aria-label={s.chapterProgress}>
      <div className={styles.prompt}>
        <span className={styles.kicker}>{s.chapterProgress}</span>
        <p className={`${styles.promptText} ${entry.completed ? styles.promptCompleted : ''}`}>
          {prompt}
        </p>
        {completedAt && (
          <span className={styles.meta}>
            <span className={styles.metaKey}>{s.completedAt}</span>
            {completedAt}
          </span>
        )}
      </div>
      <button
        type="button"
        className={`${styles.button} ${entry.completed ? styles.completed : ''}`}
        onClick={toggle}
        aria-pressed={entry.completed}>
        {buttonLabel}
      </button>
    </div>
  );
}

/** Renders under every chapter footer (via theme swizzle).
 *  Reads localStorage — client only. */
export default function ProgressToggle(): ReactNode {
  return <BrowserOnly fallback={null}>{() => <ToggleButton />}</BrowserOnly>;
}
