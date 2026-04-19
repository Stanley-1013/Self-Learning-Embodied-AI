import type {ReactNode} from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useAllDocsData} from '@docusaurus/plugin-content-docs/client';
import Link from '@docusaurus/Link';
import {useProgressMap} from '@site/src/lib/progress';
import {getStrings} from '@site/src/lib/strings';
import styles from './ProgressDashboard.module.css';

interface DocRef {
  id: string;
  path: string;
}

interface BookSummary {
  slug: string;
  docs: DocRef[];
  completed: number;
  lastCompletedAt: string | null;
}

const MAX_TICKS = 14;

function groupDocsByBook(
  docs: DocRef[],
  routeBasePath: string,
): Map<string, DocRef[]> {
  const prefix = routeBasePath.startsWith('/') ? routeBasePath : `/${routeBasePath}`;
  const map = new Map<string, DocRef[]>();
  for (const doc of docs) {
    const rel = doc.path.startsWith(prefix) ? doc.path.slice(prefix.length) : doc.path;
    const segments = rel.split('/').filter(Boolean);
    if (segments.length < 2) continue;
    const slug = segments[0];
    const list = map.get(slug) ?? [];
    list.push(doc);
    map.set(slug, list);
  }
  return map;
}

function DashboardInner(): ReactNode {
  const {i18n} = useDocusaurusContext();
  const s = getStrings(i18n.currentLocale);
  const allDocsData = useAllDocsData();
  const progressMap = useProgressMap();

  const plugin = Object.values(allDocsData)[0];
  if (!plugin) {
    return <p className={styles.empty}>{s.noDocsConfig}</p>;
  }
  const currentVersion =
    plugin.versions.find((v) => v.name === plugin.breadcrumbs) ?? plugin.versions[0];
  const docs: DocRef[] = currentVersion.docs.map((d) => ({id: d.id, path: d.path}));
  const grouped = groupDocsByBook(docs, plugin.path);

  const books: BookSummary[] = Array.from(grouped.entries())
    .map(([slug, bookDocs]) => {
      let completed = 0;
      let lastCompletedAt: string | null = null;
      for (const d of bookDocs) {
        const entry = progressMap[d.path];
        if (entry?.completed) {
          completed += 1;
          if (entry.completedAt && (!lastCompletedAt || entry.completedAt > lastCompletedAt)) {
            lastCompletedAt = entry.completedAt;
          }
        }
      }
      return {slug, docs: bookDocs, completed, lastCompletedAt};
    })
    .sort((a, b) => a.slug.localeCompare(b.slug));

  if (books.length === 0) {
    return <p className={styles.empty}>{s.noBooksYet}</p>;
  }

  const totalDocs = books.reduce((n, b) => n + b.docs.length, 0);
  const totalCompleted = books.reduce((n, b) => n + b.completed, 0);
  const overall = totalDocs === 0 ? 0 : Math.round((totalCompleted / totalDocs) * 100);
  const booksStarted = books.filter((b) => b.completed > 0).length;

  const formatDate = (iso: string) =>
    new Date(iso).toLocaleDateString(s.dateLocale, {month: '2-digit', day: '2-digit'});

  return (
    <div className={styles.wrapper}>
      <section className={styles.summaryCard} aria-label={s.totalCompletion}>
        <div className={styles.summaryLeft}>
          <div className={styles.summaryLabel}><span>{s.totalCompletion}</span></div>
          <div className={styles.summaryValue}>
            <span>{overall}</span>
            <span className={styles.summaryUnit}>%</span>
          </div>
        </div>
        <div className={styles.summaryRight}>
          <div className={styles.summaryStats}>
            <div className={styles.statBlock}>
              <span className={styles.statNum}>{totalCompleted}</span>
              <span className={styles.statLabel}>{s.chaptersCompleted}</span>
            </div>
            <div className={styles.statBlock}>
              <span className={styles.statNum}>{totalDocs}</span>
              <span className={styles.statLabel}>{s.chaptersTotal}</span>
            </div>
            <div className={styles.statBlock}>
              <span className={styles.statNum}>
                {booksStarted}
                <span style={{color: 'var(--ink-faint)', fontWeight: 400}}> / {books.length}</span>
              </span>
              <span className={styles.statLabel}>{s.booksStarted}</span>
            </div>
          </div>
          <div className={styles.progressBar} aria-hidden="true">
            <div className={styles.progressFill} style={{width: `${overall}%`}} />
          </div>
        </div>
      </section>

      <div>
        <div className={styles.sectionLabel}><span>{s.catalogLabel}</span></div>
        <h2 className={styles.sectionTitle}>{s.catalogTitle}</h2>
        <ol className={styles.bookList}>
          {books.map((book, i) => {
            const pct = book.docs.length === 0 ? 0 : Math.round((book.completed / book.docs.length) * 100);
            const ticks = Math.min(book.docs.length, MAX_TICKS);
            const ticksOn = Math.round((ticks * book.completed) / Math.max(book.docs.length, 1));
            const lastStr = book.lastCompletedAt ? formatDate(book.lastCompletedAt) : '—';
            return (
              <li key={book.slug} className={styles.bookCard}>
                <div className={styles.bookIndex}>
                  <span>Nº {String(i + 1).padStart(2, '0')}</span>
                  <span>{book.docs.length} {s.chapters}</span>
                </div>
                <div className={styles.bookHeader}>
                  <Link to={book.docs[0].path} className={styles.bookTitle}>{book.slug}</Link>
                  <span className={styles.bookPct}>{pct.toString().padStart(2, '0')}%</span>
                </div>
                <div className={styles.progressTrack} aria-hidden="true">
                  {Array.from({length: ticks}).map((_, idx) => (
                    <span key={idx} className={`${styles.tick} ${idx < ticksOn ? styles.tickOn : ''}`} />
                  ))}
                </div>
                <div className={styles.bookMeta}>
                  <span className={styles.bookMetaItem}>
                    <span className={styles.bookMetaKey}>{s.done}</span>
                    <span>{book.completed} / {book.docs.length}</span>
                  </span>
                  <span className={styles.bookMetaItem}>
                    <span className={styles.bookMetaKey}>{s.last}</span>
                    <span>{lastStr}</span>
                  </span>
                </div>
              </li>
            );
          })}
        </ol>
      </div>
    </div>
  );
}

export default function ProgressDashboard(): ReactNode {
  return (
    <BrowserOnly fallback={<p className={styles.empty}>Loading…</p>}>
      {() => <DashboardInner />}
    </BrowserOnly>
  );
}
