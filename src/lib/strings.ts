/**
 * Locale-aware UI strings for React components that render on both zh-Hant and en.
 * Markdown chapter content is handled by Docusaurus i18n (separate files per locale).
 * This module covers React component chrome only.
 */

export interface Strings {
  // ProgressToggle
  chapterProgress: string;
  promptNotDone: string;
  promptDone: string;
  btnComplete: string;
  btnCompleted: string;
  completedAt: string;
  dateLocale: string;

  // ProgressDashboard
  totalCompletion: string;
  chaptersCompleted: string;
  chaptersTotal: string;
  booksStarted: string;
  catalogLabel: string;
  catalogTitle: string;
  chapters: string;
  done: string;
  last: string;
  loading: string;
  noDocsConfig: string;
  noBooksYet: string;
}

const ZH_HANT: Strings = {
  chapterProgress: 'Chapter progress · 閱讀進度',
  promptNotDone: '讀到這裡。把它釘在閱讀進度裡。',
  promptDone: '這一章已經讀完。',
  btnComplete: '標記為已完成',
  btnCompleted: '已完成',
  completedAt: '完成於',
  dateLocale: 'zh-TW',

  totalCompletion: 'Total completion',
  chaptersCompleted: '章節已完成',
  chaptersTotal: '章節合計',
  booksStarted: '書籍已啟動',
  catalogLabel: 'Catalog · 依書籍',
  catalogTitle: '目前在架上的書',
  chapters: 'CHAPTERS',
  done: 'Done',
  last: 'Last',
  loading: '讀取進度中…',
  noDocsConfig: '找不到 docs 設定。',
  noBooksYet: '尚未偵測到任何書籍。請在 content/{book-slug}/ 下建立章節後回來。',
};

const EN: Strings = {
  chapterProgress: 'Chapter progress',
  promptNotDone: 'You made it here. Pin it to your reading log.',
  promptDone: 'This chapter is done.',
  btnComplete: 'Mark as complete',
  btnCompleted: 'Completed',
  completedAt: 'Completed at',
  dateLocale: 'en-US',

  totalCompletion: 'Total completion',
  chaptersCompleted: 'Chapters done',
  chaptersTotal: 'Total chapters',
  booksStarted: 'Books started',
  catalogLabel: 'Catalog · By book',
  catalogTitle: 'Books on the shelf',
  chapters: 'CHAPTERS',
  done: 'Done',
  last: 'Last',
  loading: 'Loading progress…',
  noDocsConfig: 'Docs configuration not found.',
  noBooksYet: 'No books detected yet. Create chapters under content/{book-slug}/ and come back.',
};

const LOCALE_MAP: Record<string, Strings> = {
  'zh-Hant': ZH_HANT,
  en: EN,
};

export function getStrings(locale: string): Strings {
  return LOCALE_MAP[locale] ?? ZH_HANT;
}
