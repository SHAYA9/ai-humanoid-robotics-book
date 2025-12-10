import React from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '../../contexts/AuthContext';

export default function ProfileNavbarItem() {
  const { user } = useAuth();

  if (user) {
    return (
      <Link
        className="navbar__item navbar__link"
        to="/profile-manager">
        Profile
      </Link>
    );
  }
  return null;
}