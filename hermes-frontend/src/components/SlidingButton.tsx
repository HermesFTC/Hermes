import { useSpring, animated } from '@react-spring/web'
import { easings } from '@react-spring/web'
import { PropsWithChildren } from 'react'
import { Outlet } from 'react-router-dom'
import { GenericButtonProps } from './GenericButton'



export default function SlidingButton({className = "", children, href = undefined, onClick = () => {}, delay = 0}: GenericButtonProps) {
  const springs = useSpring({
    config: { duration: 1000, easing: easings.easeOutCubic} ,
    delay: delay,
    from: { x: -100, opacity: 0 },
    to: { x: 0, opacity: 1 },
  })

  console.log(href)

  return (
    <animated.div
      style={{
        ...springs,
      }}
    >
        <button onClick={onClick} className={"mx-auto p-4 border-hermes-cyan-dark border-4 border-solid rounded-3xl hover:scale-125 transition duration-500 " + className}>
          { href === undefined ?
            <a>
                <h1 className="text-5xl">{children}</h1>
            </a> :
            <a href={href}>
              <h1 className="text-5xl">{children}</h1>
          </a>
          }
        </button>

    </animated.div>
  )
}