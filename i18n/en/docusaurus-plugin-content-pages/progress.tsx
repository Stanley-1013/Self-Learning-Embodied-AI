import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import ProgressDashboard from '@site/src/components/ProgressDashboard';

export default function Progress(): ReactNode {
  return (
    <Layout title="Progress" description="Learning System progress overview">
      <main className="container margin-vert--xl">
        <header style={{marginBottom: '3rem', maxWidth: '62ch'}}>
          <div
            className="label-mono"
            style={{marginBottom: '1rem', display: 'flex', alignItems: 'center', gap: '0.75rem'}}>
            <span
              aria-hidden="true"
              style={{flex: '0 0 32px', height: '1px', background: 'var(--accent)'}}
            />
            <span>Progress · Reading log</span>
          </div>
          <h1
            style={{
              fontSize: 'clamp(2rem, 1.3rem + 2vw, 3.2rem)',
              lineHeight: 1.05,
              letterSpacing: '-0.025em',
              margin: '0 0 0.8rem',
              fontWeight: 600,
            }}>
            Where are you in the book?
          </h1>
          <p style={{color: 'var(--ink-muted)', fontSize: '1.02rem', lineHeight: 1.65, margin: 0}}>
            Per-book completion ratios. Data is kept in this browser&apos;s localStorage — it never
            leaves the device; clearing site data will reset it.
          </p>
        </header>
        <ProgressDashboard />
      </main>
    </Layout>
  );
}
