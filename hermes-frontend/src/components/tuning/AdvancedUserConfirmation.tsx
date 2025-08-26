import { PropsWithChildren } from 'react';
import { ImportantInstruction } from '../TextModifications';

interface AdvancedUserConfirmationModalProps extends PropsWithChildren<any> {
  advancedUser: boolean;
  setAdvancedUser: (v: boolean) => void;
}

export const AdvancedUserConfirmationModal = ({
  advancedUser,
  setAdvancedUser,
  className="",
}: AdvancedUserConfirmationModalProps) => (
  <div className={'block ' + className}>
    <ImportantInstruction>
      <label htmlFor="advancedUser">
        I am an advanced user that understands what this does and
        would like to modify it{' '}
      </label>
    </ImportantInstruction>
    <input
      type="checkbox"
      id="advancedUser"
      checked={advancedUser}
      onChange={(e) => setAdvancedUser(e.target.checked)}
    ></input>
  </div>
);
