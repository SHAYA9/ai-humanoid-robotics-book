import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import TranslateButton from '@site/src/components/TranslateButton/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';

export default function LayoutWrapper(props) {
  return (
    <>
      <div style={{ display: 'flex', gap: '1rem', flexDirection: 'column' }}>
        <PersonalizeButton />
        <TranslateButton />
      </div>
      <Layout {...props} />
    </>
  );
}