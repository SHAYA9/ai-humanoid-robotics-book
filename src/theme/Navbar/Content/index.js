import React from 'react';
import NavbarContent from '@theme-original/Navbar/Content';
import AuthNavbarItem from '@site/src/components/auth/AuthNavbarItem';
import './navbar-content.css';

export default function NavbarContentWrapper(props) {
  return (
    <div className="navbar-content-wrapper">
      <NavbarContent {...props} />
      <AuthNavbarItem />
    </div>
  );
}
