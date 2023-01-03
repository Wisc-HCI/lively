import React from 'react';
import styled from 'styled-components';

export const Button = styled.button(props=>({
    all:'unset',
    cursor:'pointer',
    boxShadow:props.active ? 'inset 0px 0px 0px 1px #ca7ede' : 'inset 0px 0px 0px 1px #888',
    padding:8,
    margin:3,
    backgroundColor:props.active ? "#bf65d8" : '#555',
    height:30,
    borderRadius:5,
    color:'white',
    '&:hover':{
        backgroundColor:props.active ? "#ca7ede" : '#888',
        boxShadow:props.active ? 'inset 0px 0px 0px 1px #dfb1eb' : 'inset 0px 0px 0px 1px #bbb',
        color:'white',
    }
}))