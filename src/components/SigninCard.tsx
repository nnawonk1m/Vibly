import { Image, Text, TextInput, TouchableOpacity, View } from 'react-native';
import { COLORS, TYPOGRAPHY } from '../constants/theme';

const SigninCard = () => {
    return (
        <View 
        style={{
            backgroundColor: COLORS.bg,
            borderRadius: 20,
            justifyContent: 'flex-start',
            alignItems: 'center',
            width: '100%',
            maxWidth: 400,
            paddingHorizontal: 10,
            paddingTop: 16,
        }}>
            <Text 
            style={[
                TYPOGRAPHY.h1, 
                { color: COLORS.primaryText, marginTop: 10, marginBottom: 16 }
            ]} >Sign in to Vibly</Text>

            <TouchableOpacity style={{
                flexDirection: 'row',
                justifyContent: 'center',
                alignItems: 'center',
                borderColor: COLORS.primaryText,
                borderWidth: 1,
                borderRadius: 5,
                paddingHorizontal: 10,
                paddingVertical: 5,
                marginBottom: 10,
                width: '100%',
            }}>
                <Image
                    source={require('../../assets/images/google.png')}
                    style={{width: 25, height: 25}}
                />
                <Text style={[
                    TYPOGRAPHY.h2, 
                    { backgroundColor: COLORS.bg, color: COLORS.primaryText, marginLeft: 10 }
                ]}>Continue with Google</Text>
            </TouchableOpacity>

            <Text style={[ TYPOGRAPHY.h2, { color: COLORS.placeholderText } ]} >or</Text>

            <View 
            style={{
                flexDirection: 'column',
                justifyContent: 'flex-start',
                alignItems: 'flex-start',
                marginBottom: 20,
                width: '100%',
            }}>
                <Text style={[ TYPOGRAPHY.h2, { color: COLORS.primaryText }]}>Email address:</Text>
                <TextInput 
                placeholder="Enter your email address" 
                placeholderTextColor={ COLORS.placeholderText }
                style={[
                    TYPOGRAPHY.h3, 
                    { backgroundColor: COLORS.bg, 
                    color: COLORS.primaryText,
                    borderColor: COLORS.primaryText, 
                    borderWidth: 1, 
                    borderRadius: 5, 
                    width: '100%', }
                ]}
                />
            </View>

            <TouchableOpacity style={{
                backgroundColor: COLORS.primary,
                borderRadius: 10,
                alignItems: 'center',
                marginBottom: 10, 
                paddingVertical: 10,
                width: '100%',
            }}>
            <Text style={[ TYPOGRAPHY.h2, { color: COLORS.bg }]}>Continue</Text>
            </TouchableOpacity>

            <TouchableOpacity style={{
                backgroundColor: COLORS.secondary,
                flexDirection: 'row',
                justifyContent: 'center',
                alignItems: 'center',
                borderRadius: 10,
                marginBottom: 10,
                paddingVertical: 10,
                width: '100%',
            }}>
                <Text style={[ TYPOGRAPHY.h3, { color: COLORS.primaryText, marginRight: 5 }]}>Don't have an account?</Text>
                <Text style={[ TYPOGRAPHY.h3, { color: COLORS.placeholderText }]}>Sign up</Text>
            </TouchableOpacity>
        </View>
    );
};

export default SigninCard;