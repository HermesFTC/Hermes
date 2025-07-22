import React, { useMemo } from 'react'
import { useSpring, animated } from '@react-spring/web'
import { easings } from '@react-spring/web'

export function ImportantInstruction({children}) { 
    return <b className="text-red-900">{children}</b> 
}

export function AdvancedInstruction({children}) {
    return <i className="text-pink-500">{children}</i>
}

// thank you claude
// Individual fade-in item component
function FadeInItem({ children, delay }) {
    const style = useSpring({
        from: { opacity: 0, x: -100 },
        to: { opacity: 1, x: 0 },
        delay: delay,
        config: {
            duration: 1000,
            easing: easings.easeOutCubic,
        },
    });

    return <animated.div style={style}>{children}</animated.div>;
}

// Recursive function to extract all leaf nodes
function extractLeafNodes(children, path = []) {
    const leaves = [];
    
    React.Children.forEach(children, (child, index) => {
        const currentPath = [...path, index];
        
        if (typeof child === 'string' || typeof child === 'number') {
            // Text or number nodes are leaves
            leaves.push({
                content: child,
                key: currentPath.join('-'),
                isText: true
            });
        } else if (React.isValidElement(child)) {
            // Treat span tags as leaves regardless of children
            if (child.type === 'span') {
                leaves.push({
                    content: child,
                    key: currentPath.join('-'),
                    isText: false
                });
            } else if (child.props.children) {
                // If element has children, recurse into them
                const childLeaves = extractLeafNodes(child.props.children, currentPath);
                
                if (childLeaves.length === 0) {
                    // No children found, treat this element as a leaf
                    leaves.push({
                        content: child,
                        key: currentPath.join('-'),
                        isText: false
                    });
                } else {
                    // Wrap child leaves in the current element
                    childLeaves.forEach((leaf, leafIndex) => {
                        leaves.push({
                            content: React.cloneElement(child, {
                                key: `${currentPath.join('-')}-${leafIndex}`,
                                children: leaf.content
                            }),
                            key: `${currentPath.join('-')}-${leafIndex}`,
                            isText: false
                        });
                    });
                }
            } else {
                // Element with no children is a leaf
                leaves.push({
                    content: child,
                    key: currentPath.join('-'),
                    isText: false
                });
            }
        }
    });
    
    return leaves;
}

export function FadeIn({children, duration = 1000}) {
    // Recursively extract all leaf nodes
    const leafNodes = useMemo(() => extractLeafNodes(children), [children]);
    
    return (
        <>
            {leafNodes.map((leaf, index) => (
                <FadeInItem key={leaf.key} delay={index * duration}>
                    {leaf.content}
                </FadeInItem>
            ))}
        </>
    );
}