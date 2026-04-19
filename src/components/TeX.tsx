import type {ReactNode} from 'react';
import {useMemo} from 'react';
import katex from 'katex';

interface TeXProps {
  /** LaTeX source, e.g. `e^{i\pi} + 1 = 0`. */
  children: string;
  /** Render as a centered display block instead of inline. */
  block?: boolean;
}

/**
 * Render a KaTeX formula. Safe to use with dangerouslySetInnerHTML because
 * `katex.renderToString` escapes HTML characters for us, and the source
 * strings are controlled (authored in code, not user-supplied).
 *
 * KaTeX global styles are already loaded via `src/css/custom.css`.
 */
export default function TeX({children, block = false}: TeXProps): ReactNode {
  const html = useMemo(
    () =>
      katex.renderToString(children, {
        displayMode: block,
        throwOnError: false,
        strict: 'ignore',
      }),
    [children, block],
  );

  const Tag = block ? 'div' : 'span';
  return <Tag aria-label={children} dangerouslySetInnerHTML={{__html: html}} />;
}
