# Design Spec: HMI System Log and Chat Improvements

## Goal
Improve the GP4 HMI usability by migrating the low-level system events table to a tabbed pane in the right-side panel, implementing rich dynamic synthetic log generation, adding phím tắt `Shift+Enter` for prompt submissions, and integrating a "Clear History" button below the submit button.

## Proposed Changes

### 1. Unified Right-Side Panel with Tabbed Logging
We will modify the right panel to show a tabbed header:
- **Pipeline Logs**: The high-level supervisor step logs (equivalent to the old `SystemLog` sidebar contents).
- **System Events**: The low-level `taskEvents` log entries, styled as a card-based list to fit the narrow 300px sidebar width.
  - Each Event Card shows:
    - Row 1: `Timestamp` `Level` `Category / Source`
    - Row 2: **Event Name** - `detail`
    - Toggleable on click: Displays a code block showing the full formatted JSON `data`.
  - Filters above the list:
    - Select level dropdown: `ALL`, `INFO`, `WARN`, `ERR`, `DEBUG`
    - Search text input
    - Export JSON button
    - **Update (Refresh)** button: Force-reconnects the WebSocket stream and refetches the latest telemetry snapshot to resolve any connection freeze issues.

### 2. Shift+Enter Submit Mechanism
We will modify `CommandComposer.tsx` to:
- Submit the prompt when `Shift+Enter` (or `Ctrl+Enter`) is pressed.
- Allow standard newline behavior when only `Enter` is pressed.
- Update the placeholder hint to: `"Type intent in English or Vietnamese. Shift+Enter to submit · Enter for newline."`

### 3. Clear Chat History
We will add a "Clear History" button under the Submit button:
- Background: Red (`var(--color-text-danger)` / `#dc2626`)
- Text: White (`#ffffff`)
- Position: Placed below the Submit button inside a vertical flex column container `input-actions-column`.
- Behavior: When clicked, sets a `clearChatTimestamp` in state and local storage, filtering out any chat message older than this timestamp. Disabled when no messages are visible.

### 4. Remove Separate "System Log" Tab
Remove the separate "System Log" tab from `App.tsx` and only display *Command Interface* and *Joint Jog Pendant* tabs.

## Verification Plan
1. Validate CSS and JSX compile without errors.
2. Confirm the UI rendering with `browser_subagent`.
3. Verify that pressing `Shift+Enter` submits the form, and pressing `Enter` inserts a new line.
4. Verify that clicking "Clear History" hides all messages in `ChatPanel`.
5. Verify that clicking the Update/Refresh button triggers a reconnection.
