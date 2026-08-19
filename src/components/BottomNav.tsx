import { View } from 'react-native';
import { COLORS } from '../constants/theme';
import BottomNavItem from "./BottomNavItem";

type routeItem = {
    homeRoute: boolean;
    manualRoute: boolean;
}

const BottomNav = (props: routeItem ) => {
    return (
        <View style = {{
            position: 'absolute',
            bottom: 0,
            left: 0,
            right: 0,
            paddingVertical: 16,
            flexDirection: 'row',
            justifyContent: 'space-around',
            alignItems: 'center',
            backgroundColor: COLORS.primary,
            width: '100%',
        }}>
            <BottomNavItem
                name = "home"
                source = {require("../../assets/images/house-door-alt.png")}
                route = "/"
                active = {props.homeRoute}
            />
            <BottomNavItem
                name = "manual"
                source = {require("../../assets/images/wave-pulse.png")}
                route = "/manual"
                active = {props.manualRoute}
            />
        </View>
    );
};

export default BottomNav;