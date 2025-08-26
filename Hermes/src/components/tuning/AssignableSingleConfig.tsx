import { useSelector } from 'react-redux';

import BasicVariable from '@/components/views/ConfigView/BasicVariable';
import { RootState, useAppDispatch } from '@/store/reducers';
import {
  BasicVarState,
  ConfigVar,
  CustomVarState,
} from '@/store/types/config';

type SingleConfigProps = { 
  configName: string; 
  fieldName: string; 
};

const SingleConfig = ({
  configName,
  fieldName,
}: SingleConfigProps) => {
  const dispatch = useAppDispatch();

  // get all states and map to config
  const configRoot = useSelector(
    (state: RootState) => state.config.configRoot,
  ) as CustomVarState;

  // get config values
  const rootValue = configRoot.__value;
  if (rootValue === null) {
    return <div>No config loaded</div>;
  }

  // find the config group
  if (!(configName in rootValue)) {
    console.error(`Config name "${configName}" not found`);
    return <div>Config "{configName}" not found</div>;
  }

  const configGroup = rootValue[configName] as CustomVarState;
  if (configGroup.__type !== 'custom' || configGroup.__value === null) {
    console.error(`Config "${configName}" is not a valid config group`);
    return <div>Invalid config group</div>;
  }

  // find the specific field
  if (!(fieldName in configGroup.__value)) {
    console.error(`Field name "${fieldName}" not found in config "${configName}"`);
    return <div>Field "{fieldName}" not found</div>;
  }

  const fieldState = configGroup.__value[fieldName];
  if (fieldState.__type === 'custom') {
    console.error(`Field "${fieldName}" is a custom type, not a basic variable`);
    return <div>Field "{fieldName}" is not a basic variable</div>;
  }

  const basicFieldState = fieldState as BasicVarState;
  const fullPath = `${configName}.${fieldName}`;

  return (
    <BasicVariable
      key={fieldName}
      name={fieldName}
      path={fullPath}
      state={basicFieldState}
      onChange={(newState) => {
        // Update the specific field in the nested structure
        const updatedConfigGroup: CustomVarState = {
          __type: 'custom',
          __value: {
            ...configGroup.__value,
            [fieldName]: newState,
          },
        };

        const updatedRoot: CustomVarState = {
          __type: 'custom',
          __value: {
            ...rootValue,
            [configName]: updatedConfigGroup,
          },
        };

        dispatch({
          type: 'UPDATE_CONFIG',
          configRoot: updatedRoot,
        });
      }}
      onSave={(variable) => {
        // Save only the specific field that changed
        dispatch({
          type: 'SAVE_CONFIG',
          configDiff: {
            __type: 'custom',
            __value: {
              [configName]: {
                __type: 'custom',
                __value: {
                  [fieldName]: variable,
                },
              },
            },
          },
        });
      }}
    />
  );
};

export default SingleConfig;
