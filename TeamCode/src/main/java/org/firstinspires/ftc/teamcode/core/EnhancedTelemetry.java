package org.firstinspires.ftc.teamcode.core;

import androidx.annotation.Nullable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.core.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.core.OptimizationToggles.optimizeTelemetryLazyFormat;

/**
 * Telemetry implementation that fans out every call to two backends: the Driver Station
 * (HTML-formatted) and FTC Dashboard (plain text). Adds a few convenience helpers for
 * group/subgroup headers and DS-only or Dashboard-only data.
 *
 * <p>HTML/color/font constants live in {@link HtmlFormatter}; the on-screen field renderer
 * lives in {@link BrailleRenderer}.
 */
public class EnhancedTelemetry implements Telemetry {
    public static boolean enableDSTelemetry = true;
    public static boolean enableDashboardTelemetry = true;

    private final Telemetry dsTelemetry;
    private final Telemetry dashTelemetry;
    private int defaultFontSize = FONT_NORMAL;
    /** Singleton used by lazy-mode addData calls when both backends are disabled. */
    private static final Item EMPTY_ITEM = new EnhancedItem(Collections.<Item>emptyList());

    public EnhancedTelemetry(Telemetry dsTelemetry, Telemetry dashTelemetry) {
        this.dsTelemetry = dsTelemetry;
        this.dashTelemetry = dashTelemetry;
        if (enableDSTelemetry) dsTelemetry.setDisplayFormat(DisplayFormat.HTML);
    }

    public void setDSTransmissionInterval(int interval) {
        if (enableDSTelemetry) this.dsTelemetry.setMsTransmissionInterval(interval);
    }

    /** Default size for {@link #addData} formatting. Use the {@code FONT_*} constants. */
    public void setDefaultFontSize(int size) { this.defaultFontSize = size; }
    public int getDefaultFontSize() { return defaultFontSize; }

    private String fmtCaption(String caption) {
        return htmlSize(defaultFontSize, htmlBold(caption));
    }

    private String fmtValue(Object value) {
        return htmlColorSize(COLOR_VALUE, defaultFontSize, String.valueOf(value));
    }

    // ─── group / subgroup / separator ────────────────────────────────────────

    public void addGroupHeader(String groupName) {
        addGroupHeader(groupName, COLOR_MODULE);
    }

    public void addGroupHeader(String groupName, String color) {
        if (enableDSTelemetry) dsTelemetry.addLine(htmlBold(htmlColorSize(color, FONT_LARGE, groupName)));
        if (enableDashboardTelemetry) dashTelemetry.addLine("-------- " + groupName + " --------");
    }

    public void addSubgroupHeader(String subgroupName) {
        addSubgroupHeader(subgroupName, COLOR_STATE);
    }

    public void addSubgroupHeader(String subgroupName, String color) {
        if (enableDSTelemetry) dsTelemetry.addLine(htmlBold(htmlColorSize(color, FONT_NORMAL, "▸ " + subgroupName)));
        if (enableDashboardTelemetry) dashTelemetry.addLine("  » " + subgroupName);
    }

    public void addSeparator() {
        if (enableDSTelemetry) dsTelemetry.addLine(htmlColorSize(COLOR_GRAY, FONT_NORMAL, "─────────────────────────"));
        if (enableDashboardTelemetry) dashTelemetry.addLine("");
    }

    /** Add a module header to DS (colored module name + state) and plain to dashboard. */
    public void addModuleHeader(String moduleName, String stateString) {
        if (enableDSTelemetry) {
            dsTelemetry.addData(
                    htmlColor(COLOR_MODULE, htmlBold(moduleName)),
                    htmlColor(COLOR_STATE, stateString));
        }
        if (enableDashboardTelemetry) dashTelemetry.addData(moduleName, stateString);
    }

    // ─── DS-only / Dashboard-only convenience ────────────────────────────────

    public Item addDSLargeData(String caption, Object value) {
        if (!enableDSTelemetry) return new EnhancedItem(new ArrayList<>(0));
        Item it = dsTelemetry.addData(
                htmlSize(FONT_SMALL, htmlBold(caption)),
                htmlColorSize(COLOR_VALUE, FONT_XLARGE, String.valueOf(value)));
        return new EnhancedItem(singleton(it));
    }

    public EnhancedTelemetry addDSData(String caption, Object value) {
        if (enableDSTelemetry) dsTelemetry.addData(fmtCaption(caption), fmtValue(value));
        return this;
    }

    public EnhancedTelemetry addDSData(String caption, String format, Object... args) {
        if (enableDSTelemetry) dsTelemetry.addData(fmtCaption(caption), fmtValue(String.format(format, args)));
        return this;
    }

    public EnhancedTelemetry addDSLine(String value) {
        if (enableDSTelemetry) dsTelemetry.addLine(value);
        return this;
    }

    /** Add raw HTML data to the DS telemetry (caller handles formatting). */
    public EnhancedTelemetry addDSRawHtml(String caption, String htmlValue) {
        if (enableDSTelemetry) dsTelemetry.addData(fmtCaption(caption), htmlValue);
        return this;
    }

    public EnhancedTelemetry addDashboardData(String caption, Object value) {
        if (enableDashboardTelemetry) dashTelemetry.addData(caption, value);
        return this;
    }

    public EnhancedTelemetry addDashboardData(String caption, String format, Object... args) {
        // Dashboard telemetry already accepts (caption, format, args) and formats lazily, so we
        // don't need to pre-format here even when lazy mode is off.
        if (enableDashboardTelemetry) dashTelemetry.addData(caption, format, args);
        return this;
    }

    // ─── Telemetry interface (forwarded to both backends) ────────────────────

    @Override
    public Item addData(String caption, String format, Object... args) {
        if (optimizeTelemetryLazyFormat) {
            if (!enableDSTelemetry && !enableDashboardTelemetry) return EMPTY_ITEM;
            List<Item> items = new ArrayList<>(2);
            if (enableDSTelemetry) {
                items.add(dsTelemetry.addData(fmtCaption(caption), fmtValue(String.format(format, args))));
            }
            if (enableDashboardTelemetry) {
                items.add(dashTelemetry.addData(caption, format, args));
            }
            return new EnhancedItem(items);
        }
        List<Item> items = new ArrayList<>(2);
        if (enableDSTelemetry) items.add(dsTelemetry.addData(fmtCaption(caption), fmtValue(String.format(format, args))));
        if (enableDashboardTelemetry) items.add(dashTelemetry.addData(caption, format, args));
        return new EnhancedItem(items);
    }

    @Override
    public Item addData(String caption, Object value) {
        if (optimizeTelemetryLazyFormat) {
            if (!enableDSTelemetry && !enableDashboardTelemetry) return EMPTY_ITEM;
            List<Item> items = new ArrayList<>(2);
            if (enableDSTelemetry) items.add(dsTelemetry.addData(fmtCaption(caption), fmtValue(value)));
            if (enableDashboardTelemetry) items.add(dashTelemetry.addData(caption, value));
            return new EnhancedItem(items);
        }
        List<Item> items = new ArrayList<>(2);
        if (enableDSTelemetry) items.add(dsTelemetry.addData(fmtCaption(caption), fmtValue(value)));
        if (enableDashboardTelemetry) items.add(dashTelemetry.addData(caption, value));
        return new EnhancedItem(items);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        List<Item> items = new ArrayList<>(2);
        Func<String> htmlProducer = () -> fmtValue(valueProducer.value());
        if (enableDSTelemetry) items.add(dsTelemetry.addData(fmtCaption(caption), htmlProducer));
        if (enableDashboardTelemetry) items.add(dashTelemetry.addData(caption, valueProducer));
        return new EnhancedItem(items);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        List<Item> items = new ArrayList<>(2);
        Func<String> htmlProducer = () -> fmtValue(String.format(format, valueProducer.value()));
        if (enableDSTelemetry) items.add(dsTelemetry.addData(fmtCaption(caption), htmlProducer));
        if (enableDashboardTelemetry) items.add(dashTelemetry.addData(caption, format, valueProducer));
        return new EnhancedItem(items);
    }

    @Override
    public boolean removeItem(Item item) {
        boolean ds = enableDSTelemetry && dsTelemetry.removeItem(item);
        boolean dash = enableDashboardTelemetry && dashTelemetry.removeItem(item);
        return ds | dash;
    }

    @Override
    public void clear() {
        if (enableDSTelemetry) dsTelemetry.clear();
        if (enableDashboardTelemetry) dashTelemetry.clear();
    }

    @Override
    public void clearAll() {
        if (enableDSTelemetry) dsTelemetry.clearAll();
        if (enableDashboardTelemetry) dashTelemetry.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        if (enableDSTelemetry) dsTelemetry.addAction(action);
        if (enableDashboardTelemetry) dashTelemetry.addAction(action);
        return action;
    }

    @Override
    public boolean removeAction(Object token) {
        boolean ds = enableDSTelemetry && dsTelemetry.removeAction(token);
        boolean dash = enableDashboardTelemetry && dashTelemetry.removeAction(token);
        return ds | dash;
    }

    @Override
    public void speak(String text) {
        if (enableDSTelemetry) dsTelemetry.speak(text);
        if (enableDashboardTelemetry) dashTelemetry.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        if (enableDSTelemetry) dsTelemetry.speak(text, languageCode, countryCode);
        if (enableDashboardTelemetry) dashTelemetry.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        boolean ds = !enableDSTelemetry || dsTelemetry.update();
        boolean dash = !enableDashboardTelemetry || dashTelemetry.update();
        return ds & dash;
    }

    @Override
    public Line addLine() {
        List<Line> lines = new ArrayList<>(2);
        if (enableDSTelemetry) lines.add(dsTelemetry.addLine());
        if (enableDashboardTelemetry) lines.add(dashTelemetry.addLine());
        return new EnhancedLine(lines);
    }

    @Override
    public Line addLine(String lineCaption) {
        List<Line> lines = new ArrayList<>(2);
        if (enableDSTelemetry) lines.add(dsTelemetry.addLine(lineCaption));
        if (enableDashboardTelemetry) lines.add(dashTelemetry.addLine(lineCaption));
        return new EnhancedLine(lines);
    }

    @Override
    public boolean removeLine(Line line) {
        boolean ds = enableDSTelemetry && dsTelemetry.removeLine(line);
        boolean dash = enableDashboardTelemetry && dashTelemetry.removeLine(line);
        return ds | dash;
    }

    @Override
    public boolean isAutoClear() {
        if (enableDSTelemetry) return dsTelemetry.isAutoClear();
        if (enableDashboardTelemetry) return dashTelemetry.isAutoClear();
        return true;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        if (enableDSTelemetry) dsTelemetry.setAutoClear(autoClear);
        if (enableDashboardTelemetry) dashTelemetry.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        if (enableDSTelemetry) return dsTelemetry.getMsTransmissionInterval();
        if (enableDashboardTelemetry) return dashTelemetry.getMsTransmissionInterval();
        return 0;
    }

    @Override
    public void setMsTransmissionInterval(int interval) {
        if (enableDSTelemetry) dsTelemetry.setMsTransmissionInterval(interval);
        if (enableDashboardTelemetry) dashTelemetry.setMsTransmissionInterval(interval);
    }

    @Override
    public String getItemSeparator() {
        if (enableDSTelemetry) return dsTelemetry.getItemSeparator();
        if (enableDashboardTelemetry) return dashTelemetry.getItemSeparator();
        return "";
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        if (enableDSTelemetry) dsTelemetry.setItemSeparator(itemSeparator);
        if (enableDashboardTelemetry) dashTelemetry.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        if (enableDSTelemetry) return dsTelemetry.getCaptionValueSeparator();
        if (enableDashboardTelemetry) return dashTelemetry.getCaptionValueSeparator();
        return "";
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        if (enableDSTelemetry) dsTelemetry.setCaptionValueSeparator(captionValueSeparator);
        if (enableDashboardTelemetry) dashTelemetry.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        if (enableDSTelemetry) dsTelemetry.setDisplayFormat(displayFormat);
        if (enableDashboardTelemetry) dashTelemetry.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        Log dsLog = enableDSTelemetry ? dsTelemetry.log() : null;
        Log dashLog = enableDashboardTelemetry ? dashTelemetry.log() : null;
        return new CombinedLog(dsLog, dashLog);
    }

    private static <T> List<T> singleton(T value) {
        List<T> out = new ArrayList<>(1);
        out.add(value);
        return out;
    }

    // ─── inner fan-out implementations ───────────────────────────────────────

    private static class EnhancedItem implements Item {
        private final List<Item> items;
        EnhancedItem(List<Item> items) { this.items = items; }

        @Override public String getCaption() { return items.isEmpty() ? "" : items.get(0).getCaption(); }

        @Override
        public Item setCaption(String caption) {
            for (int i = 0; i < items.size(); i++) items.get(i).setCaption(caption);
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            for (int i = 0; i < items.size(); i++) items.get(i).setValue(format, args);
            return this;
        }

        @Override
        public Item setValue(Object value) {
            for (int i = 0; i < items.size(); i++) items.get(i).setValue(value);
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            for (int i = 0; i < items.size(); i++) items.get(i).setValue(valueProducer);
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            for (int i = 0; i < items.size(); i++) items.get(i).setValue(format, valueProducer);
            return this;
        }

        @Override
        public Item setRetained(@Nullable Boolean retained) {
            for (int i = 0; i < items.size(); i++) items.get(i).setRetained(retained);
            return this;
        }

        @Override public boolean isRetained() { return !items.isEmpty() && items.get(0).isRetained(); }

        @Override
        public Item addData(String caption, String format, Object... args) {
            for (int i = 0; i < items.size(); i++) items.get(i).addData(caption, format, args);
            return this;
        }

        @Override
        public Item addData(String caption, Object value) {
            for (int i = 0; i < items.size(); i++) items.get(i).addData(caption, value);
            return this;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            for (int i = 0; i < items.size(); i++) items.get(i).addData(caption, valueProducer);
            return this;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            for (int i = 0; i < items.size(); i++) items.get(i).addData(caption, format, valueProducer);
            return this;
        }
    }

    private static class EnhancedLine implements Line {
        private final List<Line> lines;
        EnhancedLine(List<Line> lines) { this.lines = lines; }

        @Override
        public Item addData(String caption, String format, Object... args) {
            List<Item> items = new ArrayList<>(lines.size());
            for (int i = 0; i < lines.size(); i++) items.add(lines.get(i).addData(caption, format, args));
            return new EnhancedItem(items);
        }

        @Override
        public Item addData(String caption, Object value) {
            List<Item> items = new ArrayList<>(lines.size());
            for (int i = 0; i < lines.size(); i++) items.add(lines.get(i).addData(caption, value));
            return new EnhancedItem(items);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            List<Item> items = new ArrayList<>(lines.size());
            for (int i = 0; i < lines.size(); i++) items.add(lines.get(i).addData(caption, valueProducer));
            return new EnhancedItem(items);
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            List<Item> items = new ArrayList<>(lines.size());
            for (int i = 0; i < lines.size(); i++) items.add(lines.get(i).addData(caption, format, valueProducer));
            return new EnhancedItem(items);
        }
    }

    private static class CombinedLog implements Log {
        @Nullable
        private final Log dsLog;
        @Nullable
        private final Log dashLog;

        CombinedLog(Log dsLog, Log dashLog) {
            this.dsLog = dsLog;
            this.dashLog = dashLog;
        }

        @Override
        public int getCapacity() {
            if (dsLog != null) return dsLog.getCapacity();
            if (dashLog != null) return dashLog.getCapacity();
            return 0;
        }

        @Override
        public void setCapacity(int capacity) {
            if (dsLog != null) dsLog.setCapacity(capacity);
            if (dashLog != null) dashLog.setCapacity(capacity);
        }

        @Override
        public DisplayOrder getDisplayOrder() {
            if (dsLog != null) return dsLog.getDisplayOrder();
            if (dashLog != null) return dashLog.getDisplayOrder();
            return DisplayOrder.OLDEST_FIRST;
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {
            if (dsLog != null) dsLog.setDisplayOrder(displayOrder);
            if (dashLog != null) dashLog.setDisplayOrder(displayOrder);
        }

        @Override
        public void add(String message) {
            if (dsLog != null) dsLog.add(message);
            if (dashLog != null) dashLog.add(message);
        }

        @Override
        public void add(String format, Object... args) {
            if (dsLog != null) dsLog.add(format, args);
            if (dashLog != null) dashLog.add(format, args);
        }

        @Override
        public void clear() {
            if (dsLog != null) dsLog.clear();
            if (dashLog != null) dashLog.clear();
        }
    }
}
