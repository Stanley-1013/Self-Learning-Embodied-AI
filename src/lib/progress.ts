import {useCallback, useEffect, useState} from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const STORAGE_KEY = 'robotic-learning.progress.v1';

export interface ProgressEntry {
  completed: boolean;
  completedAt: string | null;
  notes: string;
}

export type ProgressMap = Record<string, ProgressEntry>;

function emptyEntry(): ProgressEntry {
  return {completed: false, completedAt: null, notes: ''};
}

function readAll(): ProgressMap {
  if (!ExecutionEnvironment.canUseDOM) return {};
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) return {};
    const parsed = JSON.parse(raw) as unknown;
    if (parsed && typeof parsed === 'object') {
      return parsed as ProgressMap;
    }
    return {};
  } catch {
    return {};
  }
}

function writeAll(map: ProgressMap): void {
  if (!ExecutionEnvironment.canUseDOM) return;
  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(map));
    window.dispatchEvent(new CustomEvent('robotic-learning:progress-updated'));
  } catch {
    // localStorage 可能被禁用（私密瀏覽），靜默失敗
  }
}

export function getEntry(path: string): ProgressEntry {
  const map = readAll();
  return map[path] ?? emptyEntry();
}

export function setEntry(path: string, entry: ProgressEntry): void {
  const map = readAll();
  map[path] = entry;
  writeAll(map);
}

export function toggleCompleted(path: string): ProgressEntry {
  const current = getEntry(path);
  const next: ProgressEntry = current.completed
    ? {...current, completed: false, completedAt: null}
    : {...current, completed: true, completedAt: new Date().toISOString()};
  setEntry(path, next);
  return next;
}

/**
 * React hook：監聽單一頁面的完成狀態，並提供切換函式。
 * 跨分頁同步透過 `storage` 事件；同分頁切換由自訂事件推播。
 */
export function useProgressEntry(path: string): {
  entry: ProgressEntry;
  toggle: () => void;
} {
  const [entry, setEntryState] = useState<ProgressEntry>(emptyEntry);

  useEffect(() => {
    setEntryState(getEntry(path));
    const sync = () => setEntryState(getEntry(path));
    window.addEventListener('storage', sync);
    window.addEventListener('robotic-learning:progress-updated', sync);
    return () => {
      window.removeEventListener('storage', sync);
      window.removeEventListener('robotic-learning:progress-updated', sync);
    };
  }, [path]);

  const toggle = useCallback(() => {
    const next = toggleCompleted(path);
    setEntryState(next);
  }, [path]);

  return {entry, toggle};
}

/**
 * 讀取整個 progress map 的 hook，供 Dashboard 使用。
 */
export function useProgressMap(): ProgressMap {
  const [map, setMap] = useState<ProgressMap>({});

  useEffect(() => {
    setMap(readAll());
    const sync = () => setMap(readAll());
    window.addEventListener('storage', sync);
    window.addEventListener('robotic-learning:progress-updated', sync);
    return () => {
      window.removeEventListener('storage', sync);
      window.removeEventListener('robotic-learning:progress-updated', sync);
    };
  }, []);

  return map;
}
