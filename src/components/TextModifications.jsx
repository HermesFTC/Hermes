import React from 'react'
import { useSpring, animated, useTrail } from '@react-spring/web'
import { easings } from '@react-spring/web'
import { Spring } from '@react-spring/web'

export function ImportantInstruction({children}) { 
    return <b className="text-red-900">{children}</b> 
}

export function FadeIn({children}) {

    const items = React.Children.toArray(children);
    
    return (
        <>
            {items.map((value, index) =>
            <animated.div key={index}>
                {items[index]}
            </animated.div>
            )}
        </>
    );
}