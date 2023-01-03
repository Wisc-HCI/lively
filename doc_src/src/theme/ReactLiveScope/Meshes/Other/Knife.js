import Blade from './Blade';
import HandleL from './HandleL';
import HandleR from './HandleR';

export default function Model(props) {
  return [
      { type: 'group', children: [
        ...Blade(),
        ...HandleL(),
        ...HandleR()
      ]}
    ]
}