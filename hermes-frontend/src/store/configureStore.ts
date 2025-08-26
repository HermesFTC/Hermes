import { applyMiddleware, createStore } from 'redux';
import { createLogger } from 'redux-logger';
import thunk from 'redux-thunk';

import socketMiddleware from './middleware/socketMiddleware';
import rootReducer from './reducers';
import {
  GET_ROBOT_STATUS,
  RECEIVE_PING_TIME,
  RECEIVE_ROBOT_STATUS,
} from './types';

const HIDDEN_ACTIONS = [
  RECEIVE_PING_TIME,
  RECEIVE_ROBOT_STATUS,
  GET_ROBOT_STATUS,
];

const configureStore = () => {
  const middlewares = [
    thunk,
    socketMiddleware,
  ];

  return createStore(rootReducer, applyMiddleware(...middlewares));
};

export default configureStore;
