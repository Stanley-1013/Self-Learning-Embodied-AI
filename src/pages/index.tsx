import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import TeX from '@site/src/components/TeX';

import styles from './index.module.css';

/** Landing page — editorial reference direction.
 *  Asymmetric masthead · numbered manifesto · dense topic index · pull quote. */

interface Pillar {
  num: string;
  title: string;
  body: string;
  math: string;
}

interface Topic {
  num: string;
  topic: string;
  desc: string;
  status: 'live' | 'queued';
  statusLabel: string;
}

const PILLARS: ReadonlyArray<Pillar> = [
  {
    num: 'I',
    title: '講得清楚才算懂',
    body: '每個概念要能用兩三句精確定義，能在面試、能在和 AI 協作時直接用。含糊的「大概是…」不算懂。',
    math: '\\text{clear definition} + \\text{physical meaning}',
  },
  {
    num: 'II',
    title: '情境推理鏈',
    body: '遇到 X 情況 → 想到 Y 工具 → 因為原則 Z → 避開陷阱 W。訓練的不是公式套用，是工程判斷。',
    math: 'X \\Rightarrow Y \\;\\text{(by } Z\\text{)}',
  },
  {
    num: 'III',
    title: '閉環中定位',
    body: '每個新名詞都標出它在感知 → 規劃 → 控制閉環的哪個節點，讓概念永遠能接回具體機器人系統。',
    math: '\\text{sense} \\to \\text{plan} \\to \\text{act}',
  },
];

const TOPICS: ReadonlyArray<Topic> = [
  {num: '01–03', topic: 'C++ 系統能力', desc: '記憶體管理、多執行緒同步、原子操作與 lock-free。', status: 'live', statusLabel: '已上線'},
  {num: '04–06', topic: 'ROS 2 開發', desc: '節點通訊、TF 座標變換、QoS 與執行器進階。', status: 'live', statusLabel: '已上線'},
  {num: '07–09', topic: '運動學與動力學', desc: '正逆運動學、DH/Jacobian/奇異點、牛頓-歐拉與拉格朗日。', status: 'live', statusLabel: '已上線'},
  {num: '10–12', topic: '規劃與避障', desc: 'A*/RRT 路徑規劃、軌跡最佳化、動態環境即時重規劃。', status: 'live', statusLabel: '已上線'},
  {num: '13–16', topic: '控制與感知', desc: 'PID/阻抗/MPC 控制、視覺伺服。', status: 'live', statusLabel: '已上線'},
  {num: '17', topic: 'SLAM', desc: '未知環境同步定位與建圖。', status: 'live', statusLabel: '已上線'},
  {num: '18–20', topic: 'RL 與模仿學習', desc: 'MDP/Q-Learning → PPO/SAC → 行為克隆/DAgger。', status: 'live', statusLabel: '已上線'},
  {num: '21–22', topic: 'Sim-to-Real 與大模型', desc: '域隨機化遷移、VLA/World Model 具身決策。', status: 'live', statusLabel: '已上線'},
];

function Masthead(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const today = new Date();
  const issue = `Vol. 01 · No. ${String(today.getFullYear()).slice(-2)}.${String(today.getMonth() + 1).padStart(2, '0')}`;

  return (
    <header className={styles.masthead}>
      <div className={styles.mastheadMark} aria-hidden="true">§</div>
      <div className={styles.mastheadInner}>
        <div>
          <div className={styles.kicker}>
            <span className={styles.kickerRule} />
            <span>{issue}</span>
            <span className={styles.kickerAccent}>— 個人技術筆記</span>
          </div>
          <h1 className={styles.title}>
            從推導到
            <br />
            <span className={styles.titleAccent}>可實作的</span>
            <br />
            機器人系統。
          </h1>
          <p className={styles.lede}>
            {siteConfig.tagline}。不是教科書，也不是部落格 —
            是一本給自己用的、能追蹤進度的參考手冊。
          </p>
          <div className={styles.actions}>
            <Link className={styles.cta} to="/content/example/intro">
              進入第一章
              <span className={styles.ctaArrow} aria-hidden="true">→</span>
            </Link>
            <Link className={styles.ctaGhost} to="/progress">
              查看進度總覽
            </Link>
          </div>
        </div>
        <aside className={styles.colophon} aria-label="目次概覽">
          <span className={styles.colophonLabel}>22 章 · 全部已上線</span>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>01–03</span>
            <span className={styles.colophonName}>C++ 系統能力</span>
            <span className={styles.colophonVal}>已上線</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>04–06</span>
            <span className={styles.colophonName}>ROS 2</span>
            <span className={styles.colophonVal}>已上線</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>07–12</span>
            <span className={styles.colophonName}>運動學 · 規劃</span>
            <span className={styles.colophonVal}>已上線</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>13–17</span>
            <span className={styles.colophonName}>控制 · 感知 · SLAM</span>
            <span className={styles.colophonVal}>已上線</span>
          </div>
          <div className={styles.colophonRow}>
            <span className={styles.colophonNum}>18–22</span>
            <span className={styles.colophonName}>RL · IL · Sim2Real · VLA</span>
            <span className={styles.colophonVal}>已上線</span>
          </div>
        </aside>
      </div>
    </header>
  );
}

function Manifesto(): ReactNode {
  return (
    <section className={styles.manifesto}>
      <div className={styles.manifestoInner}>
        <div className={styles.sectionLabel}>
          <span>編輯原則 · Editorial principles</span>
        </div>
        <div className={styles.manifestoGrid}>
          {PILLARS.map((p) => (
            <article key={p.num} className={styles.manifestoItem}>
              <span className={styles.itemNum}>PRINCIPLE {p.num}</span>
              <h3 className={styles.itemTitle}>{p.title}</h3>
              <p className={styles.itemBody}>{p.body}</p>
              <div className={styles.itemMath}><TeX>{p.math}</TeX></div>
            </article>
          ))}
        </div>
      </div>
    </section>
  );
}

function Scope(): ReactNode {
  return (
    <section className={styles.scope}>
      <div className={styles.scopeInner}>
        <div className={styles.scopeIntro}>
          <div className={styles.sectionLabel} style={{marginBottom: '1.5rem'}}>
            <span>當期主題 · Index</span>
          </div>
          <h2>由此進入任何一本書。</h2>
          <p>
            每一欄代表一本正在被吸收的材料。可上線章節會標為 LIVE，
            其餘為排程中。進度與完成時間儲存於本機 localStorage。
          </p>
        </div>
        <ol className={styles.scopeList}>
          {TOPICS.map((t) => (
            <li key={t.num} className={styles.scopeRow}>
              <span className={styles.scopeNum}>{t.num}</span>
              <span className={styles.scopeTopic}>{t.topic}</span>
              <span className={styles.scopeDesc}>{t.desc}</span>
              <span
                className={`${styles.scopeStatus} ${
                  t.status === 'live' ? styles.scopeStatusLive : ''
                }`}>
                {t.statusLabel}
              </span>
            </li>
          ))}
        </ol>
      </div>
    </section>
  );
}

function Method(): ReactNode {
  return (
    <section className={styles.method}>
      <div className={styles.methodInner}>
        <p className={styles.methodQuote}>
          讀完一章，要能在白板上重現推導、畫出直覺圖，
          並講清楚它在真實系統裡代表哪一個訊號。
        </p>
        <div className={styles.methodAttribution}>
          本手冊的交付標準
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={siteConfig.title} description={siteConfig.tagline}>
      <div className={styles.page}>
        <Masthead />
        <Manifesto />
        <Scope />
        <Method />
      </div>
    </Layout>
  );
}
