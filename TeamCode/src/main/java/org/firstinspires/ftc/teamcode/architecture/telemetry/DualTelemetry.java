package org.firstinspires.ftc.teamcode.architecture.telemetry;

import androidx.annotation.Nullable;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.telemetryLazyFormat;

/** Fan-out telemetry: the Driver Station gets HTML-formatted output, FTC Dashboard gets plain text. */
public class DualTelemetry implements Telemetry {
    public static boolean enableDSTelemetry = true;
    public static boolean enableDashboardTelemetry = true;

    private final Telemetry dsTelemetry;
    private final Telemetry dashTelemetry;
    private int defaultFontSize = FONT_NORMAL;
    private boolean dsFormatApplied = false;
    private static final Item EMPTY_ITEM = new EnhancedItem(null, null);
    private static final String SEPARATOR_HTML =
            htmlColorSize(COLOR_GRAY, FONT_NORMAL, "─────────────────────────");

    public DualTelemetry(Telemetry dsTelemetry, Telemetry dashTelemetry) {
        this.dsTelemetry = dsTelemetry;
        this.dashTelemetry = dashTelemetry;
        ensureDsFormat();
    }

    // DS HTML mode is a one-shot setDisplayFormat(), so it must be re-applied on every off→on
    // toggle of enableDSTelemetry or raw HTML tags show up on the Driver Station.
    private void ensureDsFormat() {
        if (enableDSTelemetry && !dsFormatApplied) {
            dsTelemetry.setDisplayFormat(DisplayFormat.HTML);
            dsFormatApplied = true;
        } else if (!enableDSTelemetry) {
            dsFormatApplied = false;
        }
    }

    public void setDSTransmissionInterval(int interval) {
        if (enableDSTelemetry) this.dsTelemetry.setMsTransmissionInterval(interval);
    }

    public void setDefaultFontSize(int size) { this.defaultFontSize = size; }
    public int getDefaultFontSize() { return defaultFontSize; }

    private String fmtCaption(String caption) {
        return htmlSize(defaultFontSize, htmlBold(htmlEscape(caption)));
    }

    private String fmtValue(Object value) {
        return htmlColorSize(COLOR_VALUE, defaultFontSize, htmlEscape(String.valueOf(value)));
    }

    public void addGroupHeader(String groupName) {
        addGroupHeader(groupName, COLOR_MODULE);
    }

    public void addGroupHeader(String groupName, String color) {
        if (enableDSTelemetry) dsTelemetry.addLine(htmlBold(htmlColorSize(color, FONT_LARGE, htmlEscape(groupName))));
        if (enableDashboardTelemetry) dashTelemetry.addLine("-------- " + groupName + " --------");
    }

    public void addSubgroupHeader(String subgroupName) {
        addSubgroupHeader(subgroupName, COLOR_STATE);
    }

    public void addSubgroupHeader(String subgroupName, String color) {
        if (enableDSTelemetry) dsTelemetry.addLine(htmlBold(htmlColorSize(color, FONT_NORMAL, "▸ " + htmlEscape(subgroupName))));
        if (enableDashboardTelemetry) dashTelemetry.addLine("  » " + subgroupName);
    }

    public void addSeparator() {
        if (enableDSTelemetry) dsTelemetry.addLine(SEPARATOR_HTML);
        if (enableDashboardTelemetry) dashTelemetry.addLine("");
    }

    public void addModuleHeader(String moduleName, String stateString) {
        if (enableDSTelemetry) {
            dsTelemetry.addData(
                    htmlColor(COLOR_MODULE, htmlBold(htmlEscape(moduleName))),
                    htmlColor(COLOR_STATE, htmlEscape(stateString)));
        }
        if (enableDashboardTelemetry) dashTelemetry.addData(moduleName, stateString);
    }

    public Item addDSLargeData(String caption, Object value) {
        if (!enableDSTelemetry) return EMPTY_ITEM;
        Item it = dsTelemetry.addData(
                htmlSize(FONT_SMALL, htmlBold(htmlEscape(caption))),
                htmlColorSize(COLOR_VALUE, FONT_XLARGE, htmlEscape(String.valueOf(value))));
        return new EnhancedItem(it, null);
    }

    public DualTelemetry addDSData(String caption, Object value) {
        if (enableDSTelemetry) dsTelemetry.addData(fmtCaption(caption), fmtValue(value));
        return this;
    }

    public DualTelemetry addDSData(String caption, String format, Object... args) {
        if (enableDSTelemetry) dsTelemetry.addData(fmtCaption(caption), fmtValue(String.format(format, args)));
        return this;
    }

    public DualTelemetry addDSLine(String value) {
        if (enableDSTelemetry) dsTelemetry.addLine(value);
        return this;
    }

    public DualTelemetry addDSRawHtml(String caption, String htmlValue) {
        if (enableDSTelemetry) dsTelemetry.addData(fmtCaption(caption), htmlValue);
        return this;
    }

    public DualTelemetry addDashboardData(String caption, Object value) {
        if (enableDashboardTelemetry) dashTelemetry.addData(caption, value);
        return this;
    }

    public DualTelemetry addDashboardData(String caption, String format, Object... args) {
        // Dashboard formats lazily, so forward without pre-formatting.
        if (enableDashboardTelemetry) dashTelemetry.addData(caption, format, args);
        return this;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        if (telemetryLazyFormat && !enableDSTelemetry && !enableDashboardTelemetry) {
            return EMPTY_ITEM;
        }
        Item dsItem = enableDSTelemetry
                ? dsTelemetry.addData(fmtCaption(caption), fmtValue(String.format(format, args)))
                : null;
        Item dashItem = enableDashboardTelemetry
                ? dashTelemetry.addData(caption, format, args)
                : null;
        return new EnhancedItem(dsItem, dashItem);
    }

    @Override
    public Item addData(String caption, Object value) {
        if (telemetryLazyFormat && !enableDSTelemetry && !enableDashboardTelemetry) {
            return EMPTY_ITEM;
        }
        Item dsItem = enableDSTelemetry
                ? dsTelemetry.addData(fmtCaption(caption), fmtValue(value))
                : null;
        Item dashItem = enableDashboardTelemetry
                ? dashTelemetry.addData(caption, value)
                : null;
        return new EnhancedItem(dsItem, dashItem);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        if (telemetryLazyFormat && !enableDSTelemetry && !enableDashboardTelemetry) {
            return EMPTY_ITEM;
        }
        Func<String> htmlProducer = () -> fmtValue(valueProducer.value());
        Item dsItem = enableDSTelemetry
                ? dsTelemetry.addData(fmtCaption(caption), htmlProducer)
                : null;
        Item dashItem = enableDashboardTelemetry
                ? dashTelemetry.addData(caption, valueProducer)
                : null;
        return new EnhancedItem(dsItem, dashItem);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        if (telemetryLazyFormat && !enableDSTelemetry && !enableDashboardTelemetry) {
            return EMPTY_ITEM;
        }
        Func<String> htmlProducer = () -> fmtValue(String.format(format, valueProducer.value()));
        Item dsItem = enableDSTelemetry
                ? dsTelemetry.addData(fmtCaption(caption), htmlProducer)
                : null;
        Item dashItem = enableDashboardTelemetry
                ? dashTelemetry.addData(caption, format, valueProducer)
                : null;
        return new EnhancedItem(dsItem, dashItem);
    }

    @Override
    public boolean removeItem(Item item) {
        Item dsItem = item;
        Item dashItem = item;
        if (item instanceof EnhancedItem) {
            EnhancedItem wrapper = (EnhancedItem) item;
            dsItem = wrapper.ds;
            dashItem = wrapper.dash;
        }
        boolean ds = enableDSTelemetry && dsItem != null && dsTelemetry.removeItem(dsItem);
        boolean dash = enableDashboardTelemetry && dashItem != null && dashTelemetry.removeItem(dashItem);
        return ds || dash;
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
        ensureDsFormat();
        // Telemetry contract: true if transmitted. Fan-out → "any backend transmitted".
        if (!enableDSTelemetry && !enableDashboardTelemetry) return true;
        boolean ds = enableDSTelemetry && dsTelemetry.update();
        boolean dash = enableDashboardTelemetry && dashTelemetry.update();
        return ds || dash;
    }

    @Override
    public Line addLine() {
        Line dsLine = enableDSTelemetry ? dsTelemetry.addLine() : null;
        Line dashLine = enableDashboardTelemetry ? dashTelemetry.addLine() : null;
        return new EnhancedLine(dsLine, dashLine);
    }

    @Override
    public Line addLine(String lineCaption) {
        Line dsLine = enableDSTelemetry ? dsTelemetry.addLine(lineCaption) : null;
        Line dashLine = enableDashboardTelemetry ? dashTelemetry.addLine(lineCaption) : null;
        return new EnhancedLine(dsLine, dashLine);
    }

    @Override
    public boolean removeLine(Line line) {
        Line dsLine = line;
        Line dashLine = line;
        if (line instanceof EnhancedLine) {
            EnhancedLine wrapper = (EnhancedLine) line;
            dsLine = wrapper.ds;
            dashLine = wrapper.dash;
        }
        boolean ds = enableDSTelemetry && dsLine != null && dsTelemetry.removeLine(dsLine);
        boolean dash = enableDashboardTelemetry && dashLine != null && dashTelemetry.removeLine(dashLine);
        return ds || dash;
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

    private static class EnhancedItem implements Item {
        @Nullable private final Item ds;
        @Nullable private final Item dash;

        EnhancedItem(@Nullable Item ds, @Nullable Item dash) {
            this.ds = ds;
            this.dash = dash;
        }

        @Override public String getCaption() {
            if (ds != null) return ds.getCaption();
            if (dash != null) return dash.getCaption();
            return "";
        }

        @Override
        public Item setCaption(String caption) {
            if (ds != null) ds.setCaption(caption);
            if (dash != null) dash.setCaption(caption);
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            if (ds != null) ds.setValue(format, args);
            if (dash != null) dash.setValue(format, args);
            return this;
        }

        @Override
        public Item setValue(Object value) {
            if (ds != null) ds.setValue(value);
            if (dash != null) dash.setValue(value);
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            if (ds != null) ds.setValue(valueProducer);
            if (dash != null) dash.setValue(valueProducer);
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            if (ds != null) ds.setValue(format, valueProducer);
            if (dash != null) dash.setValue(format, valueProducer);
            return this;
        }

        @Override
        public Item setRetained(@Nullable Boolean retained) {
            if (ds != null) ds.setRetained(retained);
            if (dash != null) dash.setRetained(retained);
            return this;
        }

        @Override public boolean isRetained() {
            if (ds != null) return ds.isRetained();
            if (dash != null) return dash.isRetained();
            return false;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            if (ds != null) ds.addData(caption, format, args);
            if (dash != null) dash.addData(caption, format, args);
            return this;
        }

        @Override
        public Item addData(String caption, Object value) {
            if (ds != null) ds.addData(caption, value);
            if (dash != null) dash.addData(caption, value);
            return this;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            if (ds != null) ds.addData(caption, valueProducer);
            if (dash != null) dash.addData(caption, valueProducer);
            return this;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            if (ds != null) ds.addData(caption, format, valueProducer);
            if (dash != null) dash.addData(caption, format, valueProducer);
            return this;
        }
    }

    private static class EnhancedLine implements Line {
        @Nullable private final Line ds;
        @Nullable private final Line dash;

        EnhancedLine(@Nullable Line ds, @Nullable Line dash) {
            this.ds = ds;
            this.dash = dash;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            Item dsItem = ds != null ? ds.addData(caption, format, args) : null;
            Item dashItem = dash != null ? dash.addData(caption, format, args) : null;
            return new EnhancedItem(dsItem, dashItem);
        }

        @Override
        public Item addData(String caption, Object value) {
            Item dsItem = ds != null ? ds.addData(caption, value) : null;
            Item dashItem = dash != null ? dash.addData(caption, value) : null;
            return new EnhancedItem(dsItem, dashItem);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            Item dsItem = ds != null ? ds.addData(caption, valueProducer) : null;
            Item dashItem = dash != null ? dash.addData(caption, valueProducer) : null;
            return new EnhancedItem(dsItem, dashItem);
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            Item dsItem = ds != null ? ds.addData(caption, format, valueProducer) : null;
            Item dashItem = dash != null ? dash.addData(caption, format, valueProducer) : null;
            return new EnhancedItem(dsItem, dashItem);
        }
    }

    private static class CombinedLog implements Log {
        @Nullable private final Log dsLog;
        @Nullable private final Log dashLog;

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
