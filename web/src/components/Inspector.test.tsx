import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import type { NodeTypeSpec } from '../types';
import Inspector from './Inspector';

const sweep: NodeTypeSpec = {
  type: 'SweepLane',
  category: 'action',
  group: 'lane',
  help: 'Flies along the lane.',
  min_children: 0,
  max_children: 0,
  robots: ['drone'],
  params: [
    { name: 'lane', kind: 'lane', default: 1, help: 'lane number', min: 1 },
    { name: 'direction', kind: 'choice', default: 'auto', help: '', choices: ['auto', '+x', '-x'] },
    { name: 'final', kind: 'bool', default: true, help: '' },
  ],
};

describe('Inspector', () => {
  const setup = () => {
    const onChange = vi.fn();
    render(
      <Inspector
        id="n5"
        data={{ type: 'SweepLane', name: 'Lane 1: Sweep', category: 'action', params: { lane: 1, direction: 'auto', final: true } }}
        spec={sweep}
        status="RUNNING"
        issues={['Lane 1: Sweep: lane 9 does not exist: the greenhouse has 1 lane']}
        logs={[{ t: 12.5, id: 'n5', node: 'Lane 1: Sweep', msg: 'lane 1 swept', kind: 'log' }]}
        children={[]}
        onChange={onChange}
        onDelete={() => undefined}
      />,
    );
    return onChange;
  };

  it('edits the name and every kind of parameter', () => {
    const onChange = setup();
    fireEvent.change(screen.getByLabelText('name'), { target: { value: 'Sweep A' } });
    expect(onChange).toHaveBeenLastCalledWith({ name: 'Sweep A' });
    fireEvent.change(screen.getByLabelText('lane'), { target: { value: '3' } });
    expect(onChange).toHaveBeenLastCalledWith({ params: { lane: 3, direction: 'auto', final: true } });
    fireEvent.change(screen.getByLabelText('direction'), { target: { value: '-x' } });
    expect(onChange).toHaveBeenLastCalledWith({ params: { lane: 1, direction: '-x', final: true } });
    fireEvent.click(screen.getByLabelText('final'));
    expect(onChange).toHaveBeenLastCalledWith({ params: { lane: 1, direction: 'auto', final: false } });
  });

  it("shows the node's status, problems and its own log", () => {
    setup();
    expect(screen.getByText('running')).toBeTruthy();
    expect(screen.getByText(/has 1 lane/)).toBeTruthy();
    expect(screen.getByText('lane 1 swept')).toBeTruthy();
    expect(screen.getByText('0:12.5')).toBeTruthy();
  });
});
