import React from 'react';
import NavbarContent from '@theme-original/Navbar/Content';
import AuthNavbarItem from '@site/src/components/auth/AuthNavbarItem';

export default function NavbarContentWrapper(props) {
  return (
    <>
      <NavbarContent {...props} />
      <div style={{position: 'absolute', right: '6rem'}}>
        <AuthNavbarItem />
      </div>
    </>
  );
}
