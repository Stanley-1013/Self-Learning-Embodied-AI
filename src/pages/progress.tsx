import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import ProgressDashboard from '@site/src/components/ProgressDashboard';

export default function Progress(): ReactNode {
  return (
    <Layout title="進度總覽" description="學習系統進度總覽">
      <main className="container margin-vert--xl">
        <header style={{marginBottom: '3rem', maxWidth: '62ch'}}>
          <div
            className="label-mono"
            style={{marginBottom: '1rem', display: 'flex', alignItems: 'center', gap: '0.75rem'}}>
            <span
              aria-hidden="true"
              style={{flex: '0 0 32px', height: '1px', background: 'var(--accent)'}}
            />
            <span>進度總覽 · Reading log</span>
          </div>
          <h1
            style={{
              fontSize: 'clamp(2rem, 1.3rem + 2vw, 3.2rem)',
              lineHeight: 1.05,
              letterSpacing: '-0.025em',
              margin: '0 0 0.8rem',
              fontWeight: 600,
            }}>
            你讀到哪裡了？
          </h1>
          <p style={{color: 'var(--ink-muted)', fontSize: '1.02rem', lineHeight: 1.65, margin: 0}}>
            依書籍顯示章節完成比例。資料儲存在瀏覽器 localStorage，不會離開此裝置；
            清除瀏覽器資料會一併重置。
          </p>
        </header>
        <ProgressDashboard />
      </main>
    </Layout>
  );
}
