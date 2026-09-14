<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<!-- FormationPanel — mission-based formation flight (stage 1). The operator plans ONE reference route
     in the mission editor, picks a shape / spacing / altitude step here, assigns vehicles to slots,
     previews the slot layout on the map, then "Generate & upload" builds a slot-offset copy of the
     route per vehicle and uploads them (per-link serial, via the group runner); "Start all" /
     "Restart all" send the synchronized mission start. Everything goes through GroupConfirmDialog.
     See docs/02-design/features/formation-flight.design.md. -->
<script lang="ts">
  import { t } from 'svelte-i18n';
  import { untrack } from 'svelte';
  import { get } from 'svelte/store';
  import PanelShell from './panel/PanelShell.svelte';
  import Button from './panel/Button.svelte';
  import Toggle from './panel/Toggle.svelte';
  import NumberStepper from './NumberStepper.svelte';
  import GroupConfirmDialog from './GroupConfirmDialog.svelte';
  import { settings, type FormationShape } from '$lib/stores/settings';
  import { vehicles, activeVehicleId } from '$lib/stores/vehicles';
  import { allTelemetry } from '$lib/stores/telemetry';
  import { arduMission, type ArduWaypoint } from '$lib/stores/missionArdupilot';
  import { selectedVehicleIds } from '$lib/stores/fleetSelection';
  import {
    formationAssignments, formationSlotCount, formationSlotsStore, formationPreview,
    assignSlot, assignInOrder, autoAssign, clearAssignments, type FormationPreviewSlot,
  } from '$lib/stores/formation';
  import { separationAlerts } from '$lib/stores/fleetSeparation';
  import {
    formationAssembly, assemblyStatus, allInPosition, assemblyProgress, setFormationAssembly, type AssemblyTarget,
  } from '$lib/stores/formationAssembly';
  import {
    FORMATION_SHAPES, buildFormationPlan, slotPositions, minSlotSpacing, pathHeadingAt,
  } from '$lib/helpers/formationMission';
  import { cmdHasLocation } from '$lib/helpers/arduCommandCatalog';
  import { orderedVehicles, vehicleColorFor, isMavlinkVehicle, vehicleSystem, vehicleClassOf, isFixedWingClass } from '$lib/helpers/fleetStatus';
  import { isValidGpsCoordinate } from '$lib/helpers/telemetry';
  import { type GroupCommandKind, type GroupCommandParams, groupBusy, runGroupCommand, targetsFor } from '$lib/controllers/fleetControl';
  import {
    followState, followActive, followAllInPosition, engageFollow, goFollow, pauseFollow, stopFollow, setFollowSpeed, seekFollow,
  } from '$lib/controllers/formationFollow';
  import ConfirmDialog from './ConfirmDialog.svelte';
  import SegmentedToggle from './panel/SegmentedToggle.svelte';

  let dialog = $state<GroupConfirmDialog>();
  let confirmDialog = $state<ConfirmDialog>();
  let previewOn = $state(true);

  // Method: GCS trajectory follower (default — the shape is corrected continuously) or mission-based
  // (offset copies uploaded, robust to link loss).
  let method = $state<'follow' | 'mission'>('follow');
  let followSpeedMs = $state(5);
  $effect(() => { const v = followSpeedMs; untrack(() => { if (get(followState).phase !== 'idle') setFollowSpeed(v); }); });
  const fst = $derived($followState);
  const followBusy = $derived(fst.phase === 'engaging' || fst.phase === 'stopping');
  const followNotReady = $derived(fst.vehicles.filter((v) => !v.inPosition).length);
  const followEta = $derived.by(() => {
    if (fst.phase !== 'running' || fst.speedMs <= 0) return '';
    const secs = Math.max(0, (fst.lengthM - fst.s) / fst.speedMs);
    const m = Math.floor(secs / 60);
    const s = Math.round(secs % 60);
    return `${m}:${s.toString().padStart(2, '0')}`;
  });

  async function engage() {
    const base = get(arduMission);
    if (!base.length || assignedCount < 2 || !confirmDialog) return;
    const ans = await confirmDialog.show({
      title: $t('formation.follow.engageConfirmTitle'),
      message: $t('formation.follow.engageConfirmMsg', { values: { n: assignedCount } }),
      buttons: [{ label: $t('formation.follow.engageYes'), value: 'engage', danger: true }],
    });
    if (ans !== 'engage') return;
    previewOn = false; // the follower publishes the moving slot ghosts itself
    await engageFollow({
      plan: base, assignments: $formationAssignments, slots, speedMs: followSpeedMs, headingRelative: fs.headingRelative,
      turnRadiusM, maxYawRateDegS,
    });
  }
  async function followGo() {
    if (followNotReady > 0 && confirmDialog && fst.phase === 'assembling') {
      const ans = await confirmDialog.show({
        title: $t('formation.follow.go'),
        message: $t('formation.follow.notReadyGo', { values: { n: followNotReady } }),
        buttons: [{ label: $t('formation.follow.goAnyway'), value: 'go', danger: true }],
      });
      if (ans !== 'go') return;
    }
    goFollow();
  }
  async function followStop() {
    await stopFollow();
    previewOn = true;
  }
  function followErrLabel(id: string): string {
    const v = fst.vehicles.find((x) => x.vehicleId === id);
    if (!v) return '';
    if (v.linkLost) return $t('formation.follow.linkLost');
    if (v.error) return v.error;
    if (v.inPosition) return $t('formation.state.inPosition');
    return Number.isFinite(v.errorM) ? $t('formation.follow.err', { values: { d: v.errorM.toFixed(0) } }) : $t('formation.state.noTelemetry');
  }

  // Assembly stage options (session-scoped).
  let assembleOn = $state(true);
  let loiterRadiusM = $state(60);
  let commonSpeedMs = $state(0);
  let autoGo = $state(false);

  const fs = $derived($settings.formation);
  function patchFormation(p: Partial<typeof fs>) { settings.patch({ formation: { ...get(settings).formation, ...p } }); }

  // NumberStepper is bind-driven (its onchange carries the DOM event) → local mirrors, persisted on change.
  let spacingM = $state(get(settings).formation.spacingM);
  let altStepM = $state(get(settings).formation.altStepM);
  let minSeparationM = $state(get(settings).formation.minSeparationM);
  // Older saved settings predate these two keys → fall back to the defaults.
  let turnRadiusM = $state(get(settings).formation.turnRadiusM ?? 40);
  let maxYawRateDegS = $state(get(settings).formation.maxYawRateDegS ?? 12);
  let slotCountLocal = $state(get(formationSlotCount));
  $effect(() => {
    const next = { spacingM, altStepM, minSeparationM, turnRadiusM, maxYawRateDegS };
    untrack(() => {
      const cur = get(settings).formation;
      if (cur.spacingM !== next.spacingM || cur.altStepM !== next.altStepM || cur.minSeparationM !== next.minSeparationM
        || cur.turnRadiusM !== next.turnRadiusM || cur.maxYawRateDegS !== next.maxYawRateDegS) patchFormation(next);
    });
  });
  $effect(() => {
    const n = Math.max(2, Math.round(slotCountLocal));
    untrack(() => { if (get(formationSlotCount) !== n) formationSlotCount.set(n); });
  });
  // Assign actions can grow the slot count from the store side — mirror it back into the stepper.
  $effect(() => {
    const n = $formationSlotCount;
    untrack(() => { if (Math.round(slotCountLocal) !== n) slotCountLocal = n; });
  });

  const mavVehicles = $derived(orderedVehicles($vehicles).filter((v) => isMavlinkVehicle(v) && vehicleSystem(v) != null));
  const slots = $derived($formationSlotsStore);
  const assigned = $derived([...$formationAssignments.entries()].sort((a, b) => a[0] - b[0]));
  const assignedIds = $derived(assigned.map(([, id]) => id));
  const assignedCount = $derived(assignedIds.filter((id) => $vehicles.has(id)).length);

  const refPlan = $derived($arduMission);
  const refLocated = $derived(refPlan.filter((w) => cmdHasLocation(w.command) && (w.lat !== 0 || w.lon !== 0)).length);

  const spacingTooTight = $derived(slots.length >= 2 && minSlotSpacing(slots) < fs.minSeparationM);

  /** Reference point + heading for the preview: the route's first waypoint (and its path heading),
   *  else the active vehicle's position and yaw. */
  function referencePose(): { lat: number; lon: number; hDeg: number } | null {
    const plan = get(arduMission);
    const loc: number[] = [];
    plan.forEach((w, i) => { if (cmdHasLocation(w.command) && (w.lat !== 0 || w.lon !== 0)) loc.push(i); });
    if (loc.length) {
      const w = plan[loc[0]];
      const tv = get(activeVehicleId) ? get(allTelemetry).get(get(activeVehicleId)!) : undefined;
      return { lat: w.lat / 1e7, lon: w.lon / 1e7, hDeg: pathHeadingAt(plan, loc, 0, tv?.yaw ?? 0) };
    }
    const id = get(activeVehicleId);
    const tv = id ? get(allTelemetry).get(id) : undefined;
    if (tv && isValidGpsCoordinate(tv.lat, tv.lon)) return { lat: tv.lat, lon: tv.lon, hDeg: tv.yaw };
    return null;
  }

  // Map preview: recomputed whenever the layout, assignment, route or toggle changes.
  $effect(() => {
    void slots; void $formationAssignments; void $arduMission; void fs.headingRelative; void $vehicles;
    if ($followActive) return; // the follower owns the preview while it runs (moving slot ghosts)
    if (!previewOn) { formationPreview.set(null); return; }
    const pose = referencePose();
    if (!pose) { formationPreview.set(null); return; }
    const ordered = orderedVehicles($vehicles);
    const pos = slotPositions(slots, pose.lat, pose.lon, pose.hDeg, fs.headingRelative);
    const out: FormationPreviewSlot[] = pos.map((p) => {
      const vid = $formationAssignments.get(p.index) ?? null;
      const v = vid ? $vehicles.get(vid) : undefined;
      return { index: p.index, lat: p.lat, lon: p.lon, up: p.up, vehicleId: vid, color: v ? vehicleColorFor(ordered, v.vehicleId) : null, name: v?.name ?? null };
    });
    formationPreview.set(out);
    return () => formationPreview.set(null);
  });

  function selectedOrdered(): string[] {
    return mavVehicles.filter((v) => $selectedVehicleIds.has(v.vehicleId)).map((v) => v.vehicleId);
  }
  function onAssignSelected() {
    const ids = selectedOrdered();
    if (ids.length) assignInOrder(ids);
  }
  function onAutoAssign() {
    const ids = selectedOrdered();
    const pose = referencePose();
    if (!ids.length) return;
    if (!pose) { assignInOrder(ids); return; }
    autoAssign(ids, pose.lat, pose.lon, pose.hDeg);
  }

  const canUpload = $derived(assignedCount >= 2 && refLocated > 0 && !$groupBusy);
  const canStart = $derived(assignedCount >= 2 && !$groupBusy);

  async function confirmAndRun(kind: GroupCommandKind, params: GroupCommandParams = {}, onlyIds?: string[]) {
    if (!dialog || $groupBusy) return null;
    const ids = (onlyIds ?? assignedIds).filter((id) => $vehicles.has(id));
    const targets = targetsFor(ids);
    if (targets.length < 2) return null;
    const res = await dialog.show({ kind, targets, params });
    if (!res) return null;
    return runGroupCommand(kind, targetsFor(res.vehicleIds), res.params);
  }

  async function uploadFormation() {
    const base = get(arduMission);
    if (!base.length) return;
    const pose = referencePose();
    const plans: Record<string, ArduWaypoint[]> = {};
    const pendingTargets: AssemblyTarget[] = [];
    for (const [slotIdx, id] of $formationAssignments) {
      const slot = slots[slotIdx];
      const v = $vehicles.get(id);
      if (!slot || !v) continue;
      const plane = isFixedWingClass(vehicleClassOf(v));
      const b = buildFormationPlan(base, slot, fs.headingRelative, pose?.hDeg ?? 0, {
        assemble: assembleOn,
        loiterRadiusM: plane ? loiterRadiusM : 0,
        speedMs: commonSpeedMs > 0 ? commonSpeedMs : undefined,
        speedType: plane ? 0 : 1,
      });
      plans[id] = b.plan;
      if (b.assembly) {
        pendingTargets.push({
          vehicleId: id, lat: b.assembly.lat, lon: b.assembly.lon, alt: b.assembly.alt, frame: b.assembly.frame,
          loiterRadiusM: plane ? loiterRadiusM : 0, goIdx: b.goIdx, assemblyIdx: b.assemblyIdx,
        });
      }
    }
    const run = await confirmAndRun('formationUpload', { plans });
    if (!run) return;
    // Only vehicles that actually received their plan take part in the assembly monitor.
    const ok = new Set(run.items.filter((i) => i.status === 'ok').map((i) => i.vehicleId));
    const landed = pendingTargets.filter((t) => ok.has(t.vehicleId));
    setFormationAssembly(landed);
    goSent = false;
  }

  // ── Go: release the route once the slots are filled ──
  let goSent = $state(false);
  const assembled = $derived($formationAssembly);
  const notReady = $derived([...$assemblyStatus.values()].filter((s) => s.state !== 'inPosition').length);
  const canGo = $derived(!!assembled && assembled.byVehicle.size >= 2 && !$groupBusy);

  async function goFormation(skipOverride = false) {
    if (!assembled || $groupBusy) return;
    if (!skipOverride && notReady > 0 && confirmDialog) {
      const ans = await confirmDialog.show({
        title: $t('formation.goOverrideTitle'),
        message: $t('formation.goOverrideMsg', { values: { n: notReady } }),
        buttons: [{ label: $t('formation.goOverrideYes'), value: 'go', danger: true }],
      });
      if (ans !== 'go') return;
    }
    const goSeqs: Record<string, number> = {};
    for (const t of assembled.byVehicle.values()) goSeqs[t.vehicleId] = t.goIdx;
    const run = await confirmAndRun('formationGo', { goSeqs }, Object.keys(goSeqs));
    if (run) goSent = true;
  }

  // Auto-go: every slot held for 3 s → release, once per uploaded formation. The timer is cancelled
  // (effect cleanup) the moment any vehicle drops out of position.
  $effect(() => {
    const all = $allInPosition;
    if (!autoGo || goSent || !all) return;
    const id = setTimeout(() => { if (autoGo && !goSent && get(allInPosition)) void goFormation(true); }, 3000);
    return () => clearTimeout(id);
  });

  function stateLabel(id: string): string {
    const s = $assemblyStatus.get(id);
    if (!s) return '';
    return s.state === 'enRoute'
      ? $t('formation.state.enRoute', { values: { d: Number.isFinite(s.distanceM) ? s.distanceM.toFixed(0) : '?' } })
      : $t('formation.state.' + s.state);
  }

  function fmt(n: number): string { return (Math.round(n * 10) / 10).toString(); }
</script>

<PanelShell variant="compact" title={$t('formation.title')}>
  {#snippet body()}
    <div class="fm">
      <p class="fm-intro">{$t('formation.intro')}</p>

      {#if $separationAlerts.length}
        <div class="fm-alert">
          <strong>{$t('fleet.separation.title')}</strong>
          {#each $separationAlerts as a (a.a + a.b)}
            <div>{$t('fleet.separation.pair', { values: { a: a.aName, b: a.bName, d: a.distanceM.toFixed(0), p: a.projectedM.toFixed(0) } })}</div>
          {/each}
        </div>
      {/if}

      <section class="fm-sec">
        <div class="fm-row" title={method === 'follow' ? $t('formation.mode.followTip') : $t('formation.mode.missionTip')}>
          <span>{$t('formation.mode.label')}</span>
          <SegmentedToggle
            size="sm"
            disabled={$followActive || $groupBusy}
            options={[
              { value: 'follow', label: $t('formation.mode.follow'), title: $t('formation.mode.followTip') },
              { value: 'mission', label: $t('formation.mode.mission'), title: $t('formation.mode.missionTip') },
            ]}
            value={method}
            onchange={(v) => (method = v as 'follow' | 'mission')}
          />
        </div>
        <label class="fm-row">
          <span>{$t('formation.shape')}</span>
          <select class="fm-select" value={fs.shape} onchange={(e) => patchFormation({ shape: (e.currentTarget as HTMLSelectElement).value as FormationShape })}>
            {#each FORMATION_SHAPES as s (s)}
              <option value={s}>{$t('formation.shapes.' + s)}</option>
            {/each}
          </select>
        </label>
        <label class="fm-row">
          <span>{$t('formation.spacing')}</span>
          <NumberStepper bind:value={spacingM} min={2} max={500} step={1} unit="m" />
        </label>
        <label class="fm-row">
          <span>{$t('formation.altStep')}</span>
          <NumberStepper bind:value={altStepM} min={0} max={50} step={1} unit="m" />
        </label>
        <label class="fm-row" title={$t('formation.headingRelativeTip')}>
          <span>{$t('formation.headingRelative')}</span>
          <Toggle checked={fs.headingRelative} onchange={(c) => patchFormation({ headingRelative: c })} />
        </label>
        <label class="fm-row">
          <span>{$t('formation.minSeparation')}</span>
          <NumberStepper bind:value={minSeparationM} min={1} max={200} step={1} unit="m" />
        </label>
        <label class="fm-row">
          <span>{$t('formation.slotCount')}</span>
          <NumberStepper bind:value={slotCountLocal} min={2} max={16} step={1} />
        </label>
        {#if method === 'follow'}
          <label class="fm-row">
            <span>{$t('formation.follow.speed')}</span>
            <NumberStepper bind:value={followSpeedMs} min={0.5} max={30} step={0.5} decimals={1} unit="m/s" />
          </label>
          <label class="fm-row" title={$t('formation.follow.turnRadiusTip')}>
            <span>{$t('formation.follow.turnRadius')}</span>
            <NumberStepper bind:value={turnRadiusM} min={0} max={500} step={5} unit="m" disabled={$followActive} />
          </label>
          <label class="fm-row" title={$t('formation.follow.maxYawRateTip')}>
            <span>{$t('formation.follow.maxYawRate')}</span>
            <NumberStepper bind:value={maxYawRateDegS} min={2} max={60} step={1} unit="°/s" disabled={$followActive} />
          </label>
        {:else}
        <label class="fm-row" title={$t('formation.assembleTip')}>
          <span>{$t('formation.assembleBeforeStart')}</span>
          <Toggle bind:checked={assembleOn} />
        </label>
        {/if}
        {#if method === 'mission' && assembleOn}
          <label class="fm-row">
            <span>{$t('formation.loiterRadius')}</span>
            <NumberStepper bind:value={loiterRadiusM} min={20} max={500} step={10} unit="m" />
          </label>
          <label class="fm-row">
            <span>{$t('formation.commonSpeed')}</span>
            <NumberStepper bind:value={commonSpeedMs} min={0} max={60} step={0.5} decimals={1} unit="m/s" />
          </label>
        {/if}
        {#if spacingTooTight}
          <div class="fm-warn">{$t('formation.spacingWarn', { values: { s: fmt(minSlotSpacing(slots)), m: fs.minSeparationM } })}</div>
        {/if}
      </section>

      <section class="fm-sec">
        <div class="fm-sec-head">
          <span class="fm-sec-title">{$t('formation.slots')}</span>
          <div class="fm-sec-actions">
            <Button size="sm" variant="compact" disabled={$selectedVehicleIds.size === 0} title={$t('formation.assignSelectedTip')} onclick={onAssignSelected}>{$t('formation.assignSelected')}</Button>
            <Button size="sm" variant="compact" disabled={$selectedVehicleIds.size === 0} title={$t('formation.autoAssignTip')} onclick={onAutoAssign}>{$t('formation.autoAssign')}</Button>
            <Button size="sm" variant="compact" disabled={assignedCount === 0} onclick={clearAssignments}>{$t('formation.clearAssign')}</Button>
          </div>
        </div>
        <ul class="fm-slots">
          {#each slots as s (s.index)}
            {@const vid = $formationAssignments.get(s.index) ?? ''}
            <li class="fm-slot">
              <span class="fm-slot-num">{s.index + 1}</span>
              <div class="fm-slot-main">
                <div class="fm-slot-title">
                  {$t('formation.slot', { values: { n: s.index + 1 } })}
                  {#if s.forward === 0 && s.right === 0 && s.up === 0}<span class="fm-ref">{$t('formation.reference')}</span>{/if}
                </div>
                <div class="fm-slot-off">
                  {$t('formation.offsetSummary', { values: { f: fmt(s.forward), r: fmt(s.right), u: fmt(s.up) } })}
                  {#if vid && $followActive && fst.vehicles.some((x) => x.vehicleId === vid)}
                    {@const fv = fst.vehicles.find((x) => x.vehicleId === vid)}
                    <span class="fm-state" class:inPosition={fv?.inPosition} class:enRoute={fv && !fv.inPosition && !fv.linkLost && !fv.error} class:noTelemetry={fv?.linkLost || !!fv?.error}>{followErrLabel(vid)}</span>
                  {:else if vid && $assemblyStatus.has(vid)}
                    <span class="fm-state {$assemblyStatus.get(vid)?.state}">{stateLabel(vid)}</span>
                  {/if}
                </div>
              </div>
              <select class="fm-select fm-slot-sel" value={vid} onchange={(e) => assignSlot(s.index, (e.currentTarget as HTMLSelectElement).value || null)}>
                <option value="">{$t('formation.unassigned')}</option>
                {#each mavVehicles as v (v.vehicleId)}
                  <option value={v.vehicleId}>{v.name}</option>
                {/each}
              </select>
            </li>
          {/each}
        </ul>
      </section>

      <section class="fm-sec">
        <label class="fm-row">
          <span>{$t('formation.preview')}</span>
          <Toggle bind:checked={previewOn} />
        </label>
        <div class="fm-ref-plan" class:none={refLocated === 0}>
          {refLocated > 0 ? $t('formation.referencePlan', { values: { n: refLocated } }) : $t('formation.referencePlanNone')}
        </div>
        {#if assignedCount < 2}<div class="fm-hint">{$t('formation.needAssign')}</div>{/if}
      </section>
    </div>
  {/snippet}

  {#snippet footer()}
    {#if method === 'follow'}
      <div class="fm-footer">
        <div class="fm-go-status" class:ready={fst.phase === 'running'} class:none={fst.phase === 'idle'}>
          {$t('formation.follow.phase.' + fst.phase)}
          {#if fst.phase !== 'idle'} · {$t('formation.follow.progress', { values: { s: fst.s.toFixed(0), l: fst.lengthM.toFixed(0) } })}{/if}
          {#if fst.phase === 'running'} · {$t('formation.follow.pacing', { values: { v: fst.currentSpeedMs.toFixed(1), p: Math.round(fst.pace * 100), e: Number.isFinite(fst.maxErrorM) ? fst.maxErrorM.toFixed(0) : '?' } })}{/if}
          {#if followEta} · {$t('formation.follow.eta', { values: { t: followEta } })}{/if}
          {#if fst.message} · {fst.message}{/if}
        </div>
        {#if fst.phase !== 'idle'}
          <input class="fm-progress" type="range" min="0" max={Math.max(1, fst.lengthM)} step="1" value={fst.s}
                 disabled={followBusy}
                 oninput={(e) => seekFollow(Number((e.currentTarget as HTMLInputElement).value))} />
        {/if}
        {#if fst.phase === 'idle'}
          <div class="fm-hint">{$t('formation.follow.needAirborne')}</div>
          <Button variant="warning" full disabled={assignedCount < 2 || refLocated < 2 || $groupBusy} title={$t('formation.follow.engageHint')} onclick={() => { void engage(); }}>
            {$t('formation.follow.engage')}
          </Button>
        {:else}
          <div class="fm-footer-row">
            {#if fst.phase === 'assembling' || fst.phase === 'paused'}
              <Button variant={$followAllInPosition || fst.phase === 'paused' ? 'warning' : 'standard'} full disabled={followBusy} title={$t('formation.follow.goHint')} onclick={() => { void followGo(); }}>
                {fst.phase === 'paused' ? $t('formation.follow.resume') : $t('formation.follow.go')}
              </Button>
            {:else if fst.phase === 'running'}
              <Button variant="standard" full onclick={pauseFollow}>{$t('formation.follow.pause')}</Button>
            {/if}
            <Button variant="danger" full disabled={followBusy} title={$t('formation.follow.stopHint')} onclick={() => { void followStop(); }}>
              {$t('formation.follow.stop')}
            </Button>
          </div>
        {/if}
      </div>
    {:else}
    <div class="fm-footer">
      <Button variant="data" icon="upload" full disabled={!canUpload} title={$t('formation.uploadTip')} onclick={() => { void uploadFormation(); }}>{$t('formation.upload')}</Button>
      <div class="fm-footer-row">
        <Button variant="standard" full disabled={!canStart} title={assembleOn ? $t('formation.assembleHint') : $t('fleet.kindHint.missionStart')} onclick={() => { goSent = false; void confirmAndRun('missionStart'); }}>
          {assembleOn ? $t('formation.assemble') : $t('formation.start')}
        </Button>
        <Button variant="standard" full disabled={!canStart} title={$t('fleet.kindHint.missionRestart')} onclick={() => { goSent = false; void confirmAndRun('missionRestart'); }}>{$t('formation.restart')}</Button>
      </div>
      {#if assembleOn}
        <div class="fm-go">
          <div class="fm-go-status" class:ready={$allInPosition} class:none={!assembled}>
            {#if !assembled}
              {$t('formation.assemblyNone')}
            {:else}
              {$t('formation.assemblyProgress', { values: { n: $assemblyProgress[0], total: $assemblyProgress[1] } })}
              {#if notReady > 0} · {$t('formation.goWaiting', { values: { n: notReady } })}{/if}
            {/if}
          </div>
          <div class="fm-footer-row">
            <Button variant={$allInPosition ? 'warning' : 'standard'} full disabled={!canGo || goSent} title={$t('formation.goHint')} onclick={() => { void goFormation(); }}>
              {$t('formation.go')}
            </Button>
            <label class="fm-autogo" title={$t('formation.autoGoTip')}>
              <Toggle bind:checked={autoGo} disabled={!assembled} />
              <span>{$t('formation.autoGo')}</span>
            </label>
          </div>
        </div>
      {/if}
    </div>
    {/if}
  {/snippet}
</PanelShell>

<GroupConfirmDialog bind:this={dialog} />
<ConfirmDialog bind:this={confirmDialog} />

<style>
  .fm { display: flex; flex-direction: column; gap: 10px; font-size: 12px; color: #e0e0e0; }
  .fm-intro { margin: 0; font-size: 11px; color: #9a9a9a; line-height: 1.4; }
  .fm-alert {
    padding: 6px 8px;
    border-radius: 4px;
    background: rgba(224, 108, 108, 0.18);
    border: 1px solid rgba(224, 108, 108, 0.6);
    color: #f0a0a0;
    font-size: 11.5px;
    line-height: 1.4;
  }
  .fm-sec { display: flex; flex-direction: column; gap: 6px; padding-top: 8px; border-top: 1px solid #444; }
  .fm-sec:first-of-type { border-top: 0; padding-top: 0; }
  .fm-sec-head { display: flex; align-items: center; justify-content: space-between; gap: 8px; flex-wrap: wrap; }
  .fm-sec-title, .fm-row > span:first-child { font-size: 11px; font-weight: 600; letter-spacing: 0.04em; text-transform: uppercase; color: #9a9a9a; }
  .fm-sec-actions { display: flex; gap: 4px; flex-wrap: wrap; }
  .fm-row { display: flex; align-items: center; justify-content: space-between; gap: 10px; }
  .fm-select {
    min-width: 150px;
    height: 26px;
    padding: 0 8px;
    background: #434343;
    border: 1px solid #555;
    border-radius: 4px;
    color: #e0e0e0;
    font-size: 12px;
  }
  .fm-warn { padding: 4px 8px; border-radius: 4px; background: rgba(245, 166, 35, 0.15); border: 1px solid rgba(245, 166, 35, 0.5); color: #f6e3b0; font-size: 11px; }
  .fm-hint { font-size: 11px; color: #9a9a9a; }
  .fm-slots { list-style: none; margin: 0; padding: 0; display: flex; flex-direction: column; gap: 4px; }
  .fm-slot { display: flex; align-items: center; gap: 8px; padding: 4px 8px 4px 6px; background: #363636; border: 1px solid #4a4a4a; border-radius: 4px; }
  .fm-slot-num { width: 20px; height: 20px; border-radius: 50%; border: 1px dashed #37a8db; color: #37a8db; font-size: 11px; font-weight: 700; display: inline-flex; align-items: center; justify-content: center; flex: 0 0 auto; }
  .fm-slot-main { flex: 1 1 auto; min-width: 0; }
  .fm-slot-title { font-weight: 600; }
  .fm-ref { margin-left: 6px; font-size: 10px; font-weight: 600; text-transform: uppercase; color: #59aa29; }
  .fm-slot-off { font-size: 10.5px; color: #9a9a9a; font-variant-numeric: tabular-nums; }
  .fm-slot-sel { min-width: 130px; }
  .fm-ref-plan { font-size: 11px; color: #cfcfcf; }
  .fm-ref-plan.none { color: #f6e3b0; }
  .fm-footer { display: flex; flex-direction: column; gap: 6px; width: 100%; }
  .fm-footer-row { display: flex; gap: 6px; align-items: center; }
  .fm-state {
    margin-left: 8px;
    padding: 0 5px;
    border-radius: 3px;
    border: 1px solid #555;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.03em;
    color: #9a9a9a;
    font-variant-numeric: tabular-nums;
  }
  .fm-state.enRoute { border-color: #f5a623; color: #f5a623; }
  .fm-state.inPosition { border-color: #59aa29; color: #59aa29; }
  .fm-state.notArmed, .fm-state.noTelemetry { border-color: #e06c6c; color: #e06c6c; }
  .fm-go { display: flex; flex-direction: column; gap: 6px; padding-top: 6px; border-top: 1px solid #444; }
  .fm-go-status { font-size: 11px; color: #cfcfcf; font-variant-numeric: tabular-nums; }
  .fm-go-status.ready { color: #59aa29; font-weight: 600; }
  .fm-go-status.none { color: #9a9a9a; }
  .fm-autogo { display: flex; align-items: center; gap: 6px; white-space: nowrap; font-size: 11px; color: #cfcfcf; }
  .fm-progress { width: 100%; margin: 0; accent-color: #37a8db; }
</style>
