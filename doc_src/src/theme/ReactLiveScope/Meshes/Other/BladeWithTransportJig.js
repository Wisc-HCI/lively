import Blade from './Blade';
import TransportJig from './TransportJig'

export default function Model(props) {
  return [
      { type: 'group', children: [
        ...Blade(),
        ...TransportJig()
      ]}
    ]
}