import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import DashboardNavbarItem from '@site/src/components/auth/DashboardNavbarItem';
import ProfileNavbarItem from '@site/src/components/auth/ProfileNavbarItem';

export default {
  ...ComponentTypes,
  'custom-dashboard-link': DashboardNavbarItem,
  'custom-profile-link': ProfileNavbarItem,
};